/*
 * HCICS gimbal cleaner module.
 *
 * The module accepts one camera target stream. Frames with mode == 2 are
 * treated as stain targets for cleaning; other modes are kept only as the
 * latest camera snapshot for diagnostics.
 */

#include "hcics_cleaner.h"

#include "hcics_axis.h"

#include <rtdevice.h>
#include "drv_common.h"
#include <stdlib.h>

#define DBG_TAG "hcics.clean"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#ifndef HCICS_CLEANER_PWM_DEVICE_NAME
#define HCICS_CLEANER_PWM_DEVICE_NAME       "pwm3"
#endif

#ifndef HCICS_CLEANER_RELAY_PIN
#define HCICS_CLEANER_RELAY_PIN             GET_PIN(B, 12)
#endif

#ifndef HCICS_CLEANER_THREAD_STACK
#define HCICS_CLEANER_THREAD_STACK          3072
#endif

#ifndef HCICS_CLEANER_THREAD_PRIO
#define HCICS_CLEANER_THREAD_PRIO           18
#endif

#ifndef HCICS_CLEANER_TARGET_STALE_MS
#define HCICS_CLEANER_TARGET_STALE_MS       1200U
#endif

#ifndef HCICS_CLEANER_WAIT_TARGET_MS
#define HCICS_CLEANER_WAIT_TARGET_MS        6000U
#endif

#ifndef HCICS_CLEANER_AIM_TIMEOUT_MS
#define HCICS_CLEANER_AIM_TIMEOUT_MS        3000U
#endif

#ifndef HCICS_CLEANER_VERIFY_CLEAR_MS
#define HCICS_CLEANER_VERIFY_CLEAR_MS       1800U
#endif

#ifndef HCICS_CLEANER_MAX_CYCLES
#define HCICS_CLEANER_MAX_CYCLES            3U
#endif

#ifndef HCICS_CLEANER_MIN_TARGET_W
#define HCICS_CLEANER_MIN_TARGET_W          3U
#endif

#ifndef HCICS_CLEANER_MAX_TARGET_W
#define HCICS_CLEANER_MAX_TARGET_W          250U
#endif

#ifndef HCICS_CLEANER_CAMERA_CENTER_X
#define HCICS_CLEANER_CAMERA_CENTER_X       320
#endif

#ifndef HCICS_CLEANER_BRUSH_TARGET_Y
#define HCICS_CLEANER_BRUSH_TARGET_Y        350
#endif

#ifndef HCICS_CLEANER_LOCK_ERR_X
#define HCICS_CLEANER_LOCK_ERR_X            130
#endif

#ifndef HCICS_CLEANER_LOCK_ERR_Y
#define HCICS_CLEANER_LOCK_ERR_Y            55
#endif

#ifndef HCICS_CLEANER_Y_NS_PER_PX
#define HCICS_CLEANER_Y_NS_PER_PX           1000
#endif

#ifndef HCICS_CLEANER_Y_SWEEP_NS
#define HCICS_CLEANER_Y_SWEEP_NS            180000U
#endif

#ifndef HCICS_CLEANER_Z_STEP_NS
#define HCICS_CLEANER_Z_STEP_NS             12000U
#endif

#ifndef HCICS_CLEANER_SPRAY_MS
#define HCICS_CLEANER_SPRAY_MS              900U
#endif

#ifndef HCICS_CLEANER_SCRUB_MS
#define HCICS_CLEANER_SCRUB_MS              2200U
#endif

#ifndef HCICS_CLEANER_SCRUB_STEP_MS
#define HCICS_CLEANER_SCRUB_STEP_MS         160U
#endif

#ifndef HCICS_CLEANER_RETRACT_MS
#define HCICS_CLEANER_RETRACT_MS            600U
#endif

#ifndef HCICS_CLEANER_RESET_WAIT_MS
#define HCICS_CLEANER_RESET_WAIT_MS         2500U
#endif

#ifndef HCICS_CLEANER_Z_EXTEND_TIMEOUT_MS
#define HCICS_CLEANER_Z_EXTEND_TIMEOUT_MS   1800U
#endif

#ifndef HCICS_CLEANER_GUARD_STALE_MS
#define HCICS_CLEANER_GUARD_STALE_MS        300U
#endif

#ifndef HCICS_CLEANER_TOF_DANGER_MM
#define HCICS_CLEANER_TOF_DANGER_MM         50U
#endif

#ifndef HCICS_CLEANER_TOF_WASH_ENTER_MM
#define HCICS_CLEANER_TOF_WASH_ENTER_MM     69U
#endif

#ifndef HCICS_CLEANER_WASH_HARD_DANGER_MM
#define HCICS_CLEANER_WASH_HARD_DANGER_MM   46U
#endif

#if defined(RT_USING_PWM)
#include <drivers/rt_drv_pwm.h>
#define HCICS_CLEANER_HAS_PWM               1
#else
#define HCICS_CLEANER_HAS_PWM               0
#endif

typedef struct
{
    rt_bool_t initialized;
    rt_bool_t busy;
    rt_bool_t stop_requested;
    rt_bool_t relay_on;
    rt_err_t last_result;
    hcics_cleaner_state_t state;
    rt_uint8_t cycle;
    rt_uint8_t max_cycles;
    rt_uint32_t y_pulse_ns;
    rt_uint32_t z_pulse_ns;
    hcics_cleaner_config_t config;
    hcics_cleaner_camera_target_t camera;
    hcics_cleaner_guard_t guard;
    struct rt_mutex lock;
    struct rt_semaphore start_sem;
    rt_thread_t worker;
#if HCICS_CLEANER_HAS_PWM
    struct rt_device_pwm *pwm;
#endif
} hcics_cleaner_context_t;

static hcics_cleaner_context_t g_cleaner;

static rt_uint32_t hcics_cleaner_now_ms(void)
{
    return (rt_uint32_t)(((rt_uint64_t)rt_tick_get() * 1000ULL) / RT_TICK_PER_SECOND);
}

static rt_uint32_t hcics_cleaner_clamp_u32(rt_uint32_t value,
                                           rt_uint32_t min_value,
                                           rt_uint32_t max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static rt_uint32_t hcics_cleaner_move_toward(rt_uint32_t current,
                                             rt_uint32_t target,
                                             rt_uint32_t step)
{
    if (current < target)
    {
        current += step;
        if (current > target)
        {
            current = target;
        }
    }
    else if (current > target)
    {
        if ((current - target) <= step)
        {
            current = target;
        }
        else
        {
            current -= step;
        }
    }

    return current;
}

const char *hcics_cleaner_state_name(hcics_cleaner_state_t state)
{
    switch (state)
    {
    case HCICS_CLEANER_STATE_IDLE: return "IDLE";
    case HCICS_CLEANER_STATE_AIMING: return "AIMING";
    case HCICS_CLEANER_STATE_EXTENDING: return "EXTENDING";
    case HCICS_CLEANER_STATE_SPRAYING: return "SPRAYING";
    case HCICS_CLEANER_STATE_SCRUBBING: return "SCRUBBING";
    case HCICS_CLEANER_STATE_VERIFYING: return "VERIFYING";
    case HCICS_CLEANER_STATE_RETRACTING: return "RETRACTING";
    case HCICS_CLEANER_STATE_DONE: return "DONE";
    case HCICS_CLEANER_STATE_STOPPED: return "STOPPED";
    case HCICS_CLEANER_STATE_ERROR: return "ERROR";
    default: return "UNKNOWN";
    }
}

void hcics_cleaner_get_default_config(hcics_cleaner_config_t *config)
{
    if (config == RT_NULL)
    {
        return;
    }

    config->pwm_device_name = HCICS_CLEANER_PWM_DEVICE_NAME;
    config->y_pwm_channel = 1;
    config->z_pwm_channel = 2;
    config->pwm_period_ns = 20000000U;
    config->y_safe_ns = 1860000U;
    config->y_center_ns = 1550000U;
    config->y_min_ns = 270000U;
    config->y_max_ns = 1950000U;
    config->z_retract_ns = 2200000U;
    config->z_extend_ns = 1000000U;
    config->z_wash_ns = 1100000U;
    config->relay_pin = HCICS_CLEANER_RELAY_PIN;
    config->relay_on_level = PIN_LOW;
    config->relay_off_level = PIN_HIGH;
}

static void hcics_cleaner_set_state(hcics_cleaner_state_t state)
{
    rt_mutex_take(&g_cleaner.lock, RT_WAITING_FOREVER);
    g_cleaner.state = state;
    rt_mutex_release(&g_cleaner.lock);
    LOG_I("state=%s", hcics_cleaner_state_name(state));
}

static void hcics_cleaner_relay(rt_bool_t on)
{
    rt_mutex_take(&g_cleaner.lock, RT_WAITING_FOREVER);
    g_cleaner.relay_on = on ? RT_TRUE : RT_FALSE;
    rt_pin_write(g_cleaner.config.relay_pin,
                 on ? g_cleaner.config.relay_on_level :
                      g_cleaner.config.relay_off_level);
    rt_mutex_release(&g_cleaner.lock);
}

static rt_err_t hcics_cleaner_apply_pose(rt_uint32_t y_ns, rt_uint32_t z_ns)
{
    rt_err_t result;

    y_ns = hcics_cleaner_clamp_u32(y_ns,
                                   g_cleaner.config.y_min_ns,
                                   g_cleaner.config.y_max_ns);
    z_ns = hcics_cleaner_clamp_u32(z_ns,
                                   g_cleaner.config.z_extend_ns,
                                   g_cleaner.config.z_retract_ns);

    rt_mutex_take(&g_cleaner.lock, RT_WAITING_FOREVER);
    g_cleaner.y_pulse_ns = y_ns;
    g_cleaner.z_pulse_ns = z_ns;
    rt_mutex_release(&g_cleaner.lock);

#if HCICS_CLEANER_HAS_PWM
    if (g_cleaner.pwm != RT_NULL)
    {
        result = rt_pwm_set(g_cleaner.pwm, g_cleaner.config.y_pwm_channel,
                            g_cleaner.config.pwm_period_ns, y_ns);
        if (result != RT_EOK)
        {
            LOG_E("cleaner Y PWM set failed result=%d", result);
            return result;
        }
        result = rt_pwm_enable(g_cleaner.pwm, g_cleaner.config.y_pwm_channel);
        if (result != RT_EOK)
        {
            LOG_E("cleaner Y PWM enable failed result=%d", result);
            return result;
        }
        result = rt_pwm_set(g_cleaner.pwm, g_cleaner.config.z_pwm_channel,
                            g_cleaner.config.pwm_period_ns, z_ns);
        if (result != RT_EOK)
        {
            LOG_E("cleaner Z PWM set failed result=%d", result);
            return result;
        }
        result = rt_pwm_enable(g_cleaner.pwm, g_cleaner.config.z_pwm_channel);
        if (result != RT_EOK)
        {
            LOG_E("cleaner Z PWM enable failed result=%d", result);
            return result;
        }
        return RT_EOK;
    }
#endif

    LOG_E("cleaner PWM output unavailable");
    return -RT_ENOSYS;
}

static rt_err_t hcics_cleaner_safe_outputs(void)
{
    rt_err_t result;
    rt_err_t final_result = RT_EOK;

    hcics_cleaner_relay(RT_FALSE);
    result = hcics_axis_stop();
    if (result != RT_EOK)
    {
        LOG_E("axis stop failed during cleaner safe output result=%d", result);
        final_result = result;
    }

    result = hcics_cleaner_apply_pose(g_cleaner.config.y_safe_ns,
                                      g_cleaner.config.z_retract_ns);
    if ((result != RT_EOK) && (final_result == RT_EOK))
    {
        final_result = result;
    }

    return final_result;
}

static rt_bool_t hcics_cleaner_guard_forces_retract(rt_bool_t washing);

static rt_bool_t hcics_cleaner_delay_abortable(rt_uint32_t ms,
                                               rt_bool_t check_guard,
                                               rt_bool_t washing)
{
    rt_uint32_t left = ms;

    while (left > 0U)
    {
        if (g_cleaner.stop_requested)
        {
            return RT_FALSE;
        }
        if (check_guard && hcics_cleaner_guard_forces_retract(washing))
        {
            return RT_FALSE;
        }

        if (left >= 40U)
        {
            rt_thread_mdelay(40);
            left -= 40U;
        }
        else
        {
            rt_thread_mdelay(left);
            left = 0U;
        }
    }

    return RT_TRUE;
}

static rt_bool_t hcics_cleaner_wait_not_busy(rt_uint32_t timeout_ms)
{
    rt_uint32_t start_ms = hcics_cleaner_now_ms();

    while (1)
    {
        rt_bool_t busy;

        rt_mutex_take(&g_cleaner.lock, RT_WAITING_FOREVER);
        busy = g_cleaner.busy;
        rt_mutex_release(&g_cleaner.lock);

        if (!busy)
        {
            return RT_TRUE;
        }

        if ((hcics_cleaner_now_ms() - start_ms) >= timeout_ms)
        {
            return RT_FALSE;
        }

        rt_thread_mdelay(20);
    }
}

static rt_bool_t hcics_cleaner_camera_is_fresh(const hcics_cleaner_camera_target_t *target)
{
    rt_tick_t stale_ticks;

    if ((target == RT_NULL) || (!target->valid) || (target->tick == 0U))
    {
        return RT_FALSE;
    }

    stale_ticks = rt_tick_from_millisecond(HCICS_CLEANER_TARGET_STALE_MS);
    return ((rt_tick_get() - target->tick) <= stale_ticks) ? RT_TRUE : RT_FALSE;
}

static rt_bool_t hcics_cleaner_clean_frame_is_fresh(const hcics_cleaner_camera_target_t *target)
{
    rt_tick_t stale_ticks;

    if ((target == RT_NULL) ||
        (target->mode != HCICS_CLEANER_CAMERA_MODE_CLEAN) ||
        (target->tick == 0U))
    {
        return RT_FALSE;
    }

    stale_ticks = rt_tick_from_millisecond(HCICS_CLEANER_TARGET_STALE_MS);
    return ((rt_tick_get() - target->tick) <= stale_ticks) ? RT_TRUE : RT_FALSE;
}

static rt_bool_t hcics_cleaner_target_is_stain(const hcics_cleaner_camera_target_t *target)
{
    if (!hcics_cleaner_camera_is_fresh(target))
    {
        return RT_FALSE;
    }
    if (target->mode != HCICS_CLEANER_CAMERA_MODE_CLEAN)
    {
        return RT_FALSE;
    }
    if ((target->x == 0U) || (target->y == 0U) ||
        (target->x > 640U) || (target->y > 480U))
    {
        return RT_FALSE;
    }
    if ((target->width < HCICS_CLEANER_MIN_TARGET_W) ||
        (target->width > HCICS_CLEANER_MAX_TARGET_W))
    {
        return RT_FALSE;
    }

    return RT_TRUE;
}

static rt_bool_t hcics_cleaner_get_clean_frame(hcics_cleaner_camera_target_t *target)
{
    hcics_cleaner_camera_target_t snapshot;

    if (target == RT_NULL)
    {
        return RT_FALSE;
    }

    rt_mutex_take(&g_cleaner.lock, RT_WAITING_FOREVER);
    snapshot = g_cleaner.camera;
    rt_mutex_release(&g_cleaner.lock);

    if (!hcics_cleaner_clean_frame_is_fresh(&snapshot))
    {
        return RT_FALSE;
    }

    *target = snapshot;
    return RT_TRUE;
}

static rt_bool_t hcics_cleaner_get_stain_target(hcics_cleaner_camera_target_t *target)
{
    hcics_cleaner_camera_target_t snapshot;

    if (target == RT_NULL)
    {
        return RT_FALSE;
    }

    rt_mutex_take(&g_cleaner.lock, RT_WAITING_FOREVER);
    snapshot = g_cleaner.camera;
    rt_mutex_release(&g_cleaner.lock);

    if (!hcics_cleaner_target_is_stain(&snapshot))
    {
        return RT_FALSE;
    }

    *target = snapshot;
    return RT_TRUE;
}

static rt_err_t hcics_cleaner_wait_stain(hcics_cleaner_camera_target_t *target,
                                         rt_uint32_t timeout_ms)
{
    rt_tick_t start = rt_tick_get();
    rt_tick_t timeout = rt_tick_from_millisecond(timeout_ms);

    while (!g_cleaner.stop_requested)
    {
        if (hcics_cleaner_get_stain_target(target))
        {
            return RT_EOK;
        }

        if ((rt_tick_get() - start) >= timeout)
        {
            return -RT_ETIMEOUT;
        }

        rt_thread_mdelay(40);
    }

    return -RT_EINTR;
}

static rt_uint32_t hcics_cleaner_y_from_target(const hcics_cleaner_camera_target_t *target)
{
    int err_y;
    int y_ns;

    if (target == RT_NULL)
    {
        return g_cleaner.config.y_center_ns;
    }

    err_y = (int)target->y - HCICS_CLEANER_BRUSH_TARGET_Y;
    y_ns = (int)g_cleaner.config.y_center_ns +
           (err_y * HCICS_CLEANER_Y_NS_PER_PX);

    return hcics_cleaner_clamp_u32((rt_uint32_t)y_ns,
                                   g_cleaner.config.y_min_ns,
                                   g_cleaner.config.y_max_ns);
}

static rt_bool_t hcics_cleaner_target_locked(const hcics_cleaner_camera_target_t *target)
{
    int err_x;
    int err_y;

    if (target == RT_NULL)
    {
        return RT_FALSE;
    }

    err_x = (int)target->x - HCICS_CLEANER_CAMERA_CENTER_X;
    err_y = (int)target->y - HCICS_CLEANER_BRUSH_TARGET_Y;

    if (err_x < 0)
    {
        err_x = -err_x;
    }
    if (err_y < 0)
    {
        err_y = -err_y;
    }

    return ((err_x <= HCICS_CLEANER_LOCK_ERR_X) &&
            (err_y <= HCICS_CLEANER_LOCK_ERR_Y)) ? RT_TRUE : RT_FALSE;
}

static rt_err_t hcics_cleaner_aim(hcics_cleaner_camera_target_t *locked)
{
    rt_tick_t start = rt_tick_get();
    rt_tick_t timeout = rt_tick_from_millisecond(HCICS_CLEANER_AIM_TIMEOUT_MS);
    rt_uint8_t stable = 0U;

    while (!g_cleaner.stop_requested)
    {
        hcics_cleaner_camera_target_t target;

        if (hcics_cleaner_get_stain_target(&target))
        {
            rt_uint32_t y_ns = hcics_cleaner_y_from_target(&target);
            int err_x = (int)target.x - HCICS_CLEANER_CAMERA_CENTER_X;
            rt_err_t result;

            result = hcics_cleaner_apply_pose(y_ns, g_cleaner.config.z_retract_ns);
            if (result != RT_EOK)
            {
                return result;
            }
            result = hcics_axis_center_from_error(err_x);
            if (result != RT_EOK)
            {
                return result;
            }
            if (hcics_cleaner_target_locked(&target))
            {
                stable++;
                if (stable >= 3U)
                {
                    if (locked != RT_NULL)
                    {
                        *locked = target;
                    }
                    return RT_EOK;
                }
            }
            else
            {
                stable = 0U;
            }
        }
        else
        {
            stable = 0U;
        }

        if ((rt_tick_get() - start) >= timeout)
        {
            return -RT_ETIMEOUT;
        }

        rt_thread_mdelay(80);
    }

    return -RT_EINTR;
}

static hcics_cleaner_guard_t hcics_cleaner_get_guard(void)
{
    hcics_cleaner_guard_t guard;

    rt_mutex_take(&g_cleaner.lock, RT_WAITING_FOREVER);
    guard = g_cleaner.guard;
    rt_mutex_release(&g_cleaner.lock);

    return guard;
}

static rt_bool_t hcics_cleaner_guard_is_fresh(const hcics_cleaner_guard_t *guard)
{
    rt_tick_t stale_ticks;

    if ((guard == RT_NULL) || (guard->tick == 0U))
    {
        return RT_FALSE;
    }

    stale_ticks = rt_tick_from_millisecond(HCICS_CLEANER_GUARD_STALE_MS);
    return ((rt_tick_get() - guard->tick) <= stale_ticks) ? RT_TRUE : RT_FALSE;
}

static rt_bool_t hcics_cleaner_guard_forces_retract(rt_bool_t washing)
{
    hcics_cleaner_guard_t guard = hcics_cleaner_get_guard();
    rt_bool_t fresh = hcics_cleaner_guard_is_fresh(&guard);

    if (guard.force_stop)
    {
        return RT_TRUE;
    }

    if (!fresh)
    {
        LOG_E("cleaner guard stale");
        return RT_TRUE;
    }

    if (!guard.tof_valid)
    {
        LOG_E("cleaner guard invalid");
        return RT_TRUE;
    }

    if (fresh && guard.tof_valid)
    {
        if (guard.tof_mm <= HCICS_CLEANER_WASH_HARD_DANGER_MM)
        {
            return RT_TRUE;
        }
        if (!washing && (guard.tof_mm <= HCICS_CLEANER_TOF_DANGER_MM))
        {
            return RT_TRUE;
        }
    }

    return RT_FALSE;
}

static rt_bool_t hcics_cleaner_surface_reached(void)
{
    hcics_cleaner_guard_t guard = hcics_cleaner_get_guard();

    if (!hcics_cleaner_guard_is_fresh(&guard))
    {
        return RT_FALSE;
    }

    if (guard.contact)
    {
        return RT_TRUE;
    }
    if (guard.tof_valid && (guard.tof_mm <= HCICS_CLEANER_TOF_WASH_ENTER_MM))
    {
        return RT_TRUE;
    }

    return RT_FALSE;
}

static rt_err_t hcics_cleaner_extend_z(rt_uint32_t y_ns)
{
    rt_uint32_t z_ns = g_cleaner.config.z_retract_ns;
    rt_uint32_t start_ms = hcics_cleaner_now_ms();

    while (!g_cleaner.stop_requested)
    {
        hcics_cleaner_camera_target_t target;

        if (hcics_cleaner_guard_forces_retract(RT_FALSE))
        {
            return -RT_EINTR;
        }

        if (hcics_cleaner_get_stain_target(&target))
        {
            y_ns = hcics_cleaner_y_from_target(&target);
        }

        z_ns = hcics_cleaner_move_toward(z_ns,
                                         g_cleaner.config.z_wash_ns,
                                         HCICS_CLEANER_Z_STEP_NS);
        {
            rt_err_t result = hcics_cleaner_apply_pose(y_ns, z_ns);
            if (result != RT_EOK)
            {
                return result;
            }
        }

        if (hcics_cleaner_surface_reached())
        {
            LOG_I("surface reached by guard");
            return RT_EOK;
        }
        if (z_ns == g_cleaner.config.z_wash_ns)
        {
            LOG_E("z wash pose reached without fresh guard surface confirmation");
            return -RT_ERROR;
        }
        if ((hcics_cleaner_now_ms() - start_ms) >= HCICS_CLEANER_Z_EXTEND_TIMEOUT_MS)
        {
            LOG_E("z extend timeout without fresh guard surface confirmation");
            return -RT_ETIMEOUT;
        }

        rt_thread_mdelay(30);
    }

    return -RT_EINTR;
}

static rt_err_t hcics_cleaner_spray(void)
{
    rt_err_t result = RT_EOK;

    if (hcics_cleaner_guard_forces_retract(RT_TRUE))
    {
        return -RT_EINTR;
    }

    hcics_cleaner_relay(RT_TRUE);
    if (!hcics_cleaner_delay_abortable(HCICS_CLEANER_SPRAY_MS, RT_TRUE, RT_TRUE))
    {
        result = -RT_EINTR;
    }
    hcics_cleaner_relay(RT_FALSE);

    if (result != RT_EOK)
    {
        return result;
    }

    return hcics_cleaner_guard_forces_retract(RT_TRUE) ? -RT_EINTR : RT_EOK;
}

static rt_err_t hcics_cleaner_scrub(rt_uint32_t base_y_ns)
{
    rt_uint32_t start_ms = hcics_cleaner_now_ms();
    rt_bool_t right = RT_TRUE;

    hcics_cleaner_relay(RT_TRUE);
    while (!g_cleaner.stop_requested)
    {
        rt_uint32_t y_ns;

        if (hcics_cleaner_guard_forces_retract(RT_TRUE))
        {
            return -RT_EINTR;
        }

        y_ns = right ? (base_y_ns + HCICS_CLEANER_Y_SWEEP_NS) :
                       (base_y_ns - HCICS_CLEANER_Y_SWEEP_NS);
        y_ns = hcics_cleaner_clamp_u32(y_ns,
                                       g_cleaner.config.y_min_ns,
                                       g_cleaner.config.y_max_ns);
        {
            rt_err_t result = hcics_cleaner_apply_pose(y_ns, g_cleaner.config.z_wash_ns);
            if (result != RT_EOK)
            {
                return result;
            }
            result = hcics_axis_sweep_step(right);
            if (result != RT_EOK)
            {
                return result;
            }
        }
        right = right ? RT_FALSE : RT_TRUE;

        if ((hcics_cleaner_now_ms() - start_ms) >= HCICS_CLEANER_SCRUB_MS)
        {
            return RT_EOK;
        }

        if (!hcics_cleaner_delay_abortable(HCICS_CLEANER_SCRUB_STEP_MS,
                                           RT_TRUE,
                                           RT_TRUE))
        {
            return -RT_EINTR;
        }
    }

    return -RT_EINTR;
}

static rt_err_t hcics_cleaner_retract(void)
{
    rt_uint32_t z_ns;
    rt_err_t result;

    hcics_cleaner_set_state(HCICS_CLEANER_STATE_RETRACTING);
    hcics_cleaner_relay(RT_FALSE);
    result = hcics_axis_stop();
    if (result != RT_EOK)
    {
        return result;
    }

    z_ns = g_cleaner.z_pulse_ns;
    while (z_ns != g_cleaner.config.z_retract_ns)
    {
        z_ns = hcics_cleaner_move_toward(z_ns,
                                         g_cleaner.config.z_retract_ns,
                                         HCICS_CLEANER_Z_STEP_NS * 2U);
        result = hcics_cleaner_apply_pose(g_cleaner.config.y_safe_ns, z_ns);
        if (result != RT_EOK)
        {
            return result;
        }
        rt_thread_mdelay(25);
    }

    (void)hcics_cleaner_delay_abortable(HCICS_CLEANER_RETRACT_MS,
                                        RT_FALSE,
                                        RT_FALSE);
    return RT_EOK;
}

static rt_err_t hcics_cleaner_verify_clear(void)
{
    rt_tick_t start = rt_tick_get();
    rt_tick_t clear_start = start;
    rt_tick_t clear_ticks = rt_tick_from_millisecond(HCICS_CLEANER_VERIFY_CLEAR_MS);
    rt_tick_t timeout = rt_tick_from_millisecond(HCICS_CLEANER_WAIT_TARGET_MS);
    rt_bool_t clear_active = RT_FALSE;

    while (!g_cleaner.stop_requested)
    {
        hcics_cleaner_camera_target_t target;

        if (hcics_cleaner_guard_forces_retract(RT_TRUE))
        {
            return -RT_EINTR;
        }

        if (hcics_cleaner_get_clean_frame(&target))
        {
            if (hcics_cleaner_target_is_stain(&target))
            {
                clear_active = RT_FALSE;
                clear_start = rt_tick_get();
            }
            else
            {
                if (!clear_active)
                {
                    clear_active = RT_TRUE;
                    clear_start = rt_tick_get();
                }

                if ((rt_tick_get() - clear_start) >= clear_ticks)
                {
                    return RT_EOK;
                }
            }
        }
        else
        {
            clear_active = RT_FALSE;
        }

        if ((rt_tick_get() - start) >= timeout)
        {
            LOG_E("mode=2 clear verify timeout");
            return -RT_ETIMEOUT;
        }

        rt_thread_mdelay(80);
    }

    return -RT_EINTR;
}

static rt_err_t hcics_cleaner_run_once(void)
{
    hcics_cleaner_camera_target_t target;
    rt_uint32_t base_y_ns;
    rt_err_t result;

    hcics_cleaner_set_state(HCICS_CLEANER_STATE_AIMING);
    result = hcics_cleaner_wait_stain(&target, HCICS_CLEANER_WAIT_TARGET_MS);
    if (result == -RT_ETIMEOUT)
    {
        LOG_E("no mode=2 stain target; cleaning cannot start");
        return result;
    }
    if (result != RT_EOK)
    {
        return result;
    }

    result = hcics_cleaner_aim(&target);
    if (result != RT_EOK)
    {
        return result;
    }

    base_y_ns = hcics_cleaner_y_from_target(&target);
    hcics_cleaner_set_state(HCICS_CLEANER_STATE_EXTENDING);
    result = hcics_cleaner_extend_z(base_y_ns);
    if (result != RT_EOK)
    {
        return result;
    }

    hcics_cleaner_set_state(HCICS_CLEANER_STATE_SPRAYING);
    result = hcics_cleaner_spray();
    if (result != RT_EOK)
    {
        return result;
    }

    hcics_cleaner_set_state(HCICS_CLEANER_STATE_SCRUBBING);
    result = hcics_cleaner_scrub(base_y_ns);
    hcics_cleaner_relay(RT_FALSE);
    if (result != RT_EOK)
    {
        return result;
    }

    result = hcics_cleaner_retract();
    if (result != RT_EOK)
    {
        return result;
    }
    hcics_cleaner_set_state(HCICS_CLEANER_STATE_VERIFYING);

    return hcics_cleaner_verify_clear();
}

static rt_err_t hcics_cleaner_run_mission(void)
{
    rt_err_t result = RT_EOK;
    rt_uint8_t cycle;
    rt_uint8_t max_cycles;

    rt_mutex_take(&g_cleaner.lock, RT_WAITING_FOREVER);
    max_cycles = g_cleaner.max_cycles;
    rt_mutex_release(&g_cleaner.lock);

    for (cycle = 0U; cycle < max_cycles; cycle++)
    {
        rt_mutex_take(&g_cleaner.lock, RT_WAITING_FOREVER);
        g_cleaner.cycle = (rt_uint8_t)(cycle + 1U);
        rt_mutex_release(&g_cleaner.lock);

        result = hcics_cleaner_run_once();
        if (result == RT_EOK)
        {
            return RT_EOK;
        }
        if (result == -RT_EINTR)
        {
            return result;
        }

        LOG_W("clean cycle %u failed result=%d, retry", cycle + 1U, result);
        result = hcics_cleaner_retract();
        if (result != RT_EOK)
        {
            return result;
        }
    }

    return -RT_ERROR;
}

static void hcics_cleaner_worker(void *parameter)
{
    RT_UNUSED(parameter);

    while (1)
    {
        rt_err_t result;
        rt_bool_t stopped;

        if (rt_sem_take(&g_cleaner.start_sem, RT_WAITING_FOREVER) != RT_EOK)
        {
            continue;
        }

        result = hcics_cleaner_safe_outputs();
        if (result == RT_EOK)
        {
            result = hcics_cleaner_run_mission();
        }

        rt_mutex_take(&g_cleaner.lock, RT_WAITING_FOREVER);
        stopped = g_cleaner.stop_requested;
        rt_mutex_release(&g_cleaner.lock);

        (void)hcics_cleaner_safe_outputs();

        rt_mutex_take(&g_cleaner.lock, RT_WAITING_FOREVER);
        g_cleaner.last_result = result;
        g_cleaner.busy = RT_FALSE;
        if (stopped)
        {
            g_cleaner.state = HCICS_CLEANER_STATE_STOPPED;
        }
        else if (result == RT_EOK)
        {
            g_cleaner.state = HCICS_CLEANER_STATE_DONE;
        }
        else
        {
            g_cleaner.state = HCICS_CLEANER_STATE_ERROR;
        }
        rt_mutex_release(&g_cleaner.lock);

        LOG_I("mission finished state=%s result=%d",
              hcics_cleaner_state_name(g_cleaner.state), result);
    }
}

static void hcics_cleaner_sanitize_config(hcics_cleaner_config_t *config)
{
    if (config->pwm_device_name == RT_NULL)
    {
        config->pwm_device_name = HCICS_CLEANER_PWM_DEVICE_NAME;
    }
    if (config->y_pwm_channel == 0)
    {
        config->y_pwm_channel = 1;
    }
    if (config->z_pwm_channel == 0)
    {
        config->z_pwm_channel = 2;
    }
    if (config->pwm_period_ns == 0U)
    {
        config->pwm_period_ns = 20000000U;
    }
    if (config->y_min_ns > config->y_max_ns)
    {
        rt_uint32_t tmp = config->y_min_ns;
        config->y_min_ns = config->y_max_ns;
        config->y_max_ns = tmp;
    }
    if (config->z_extend_ns > config->z_retract_ns)
    {
        rt_uint32_t tmp = config->z_extend_ns;
        config->z_extend_ns = config->z_retract_ns;
        config->z_retract_ns = tmp;
    }

    config->y_safe_ns = hcics_cleaner_clamp_u32(config->y_safe_ns,
                                                config->y_min_ns,
                                                config->y_max_ns);
    config->y_center_ns = hcics_cleaner_clamp_u32(config->y_center_ns,
                                                  config->y_min_ns,
                                                  config->y_max_ns);
    config->z_wash_ns = hcics_cleaner_clamp_u32(config->z_wash_ns,
                                                config->z_extend_ns,
                                                config->z_retract_ns);
}

rt_err_t hcics_cleaner_init(const hcics_cleaner_config_t *config)
{
    hcics_cleaner_config_t local_config;
    rt_err_t result;

    if (g_cleaner.initialized)
    {
        return RT_EOK;
    }

    if (config == RT_NULL)
    {
        hcics_cleaner_get_default_config(&local_config);
    }
    else
    {
        local_config = *config;
    }
    hcics_cleaner_sanitize_config(&local_config);

    rt_memset(&g_cleaner, 0, sizeof(g_cleaner));
    g_cleaner.config = local_config;
    g_cleaner.state = HCICS_CLEANER_STATE_IDLE;
    g_cleaner.max_cycles = HCICS_CLEANER_MAX_CYCLES;
    g_cleaner.y_pulse_ns = g_cleaner.config.y_safe_ns;
    g_cleaner.z_pulse_ns = g_cleaner.config.z_retract_ns;
    g_cleaner.guard.tof_mm = HCICS_CLEANER_TOF_INVALID_MM;
    g_cleaner.guard.tof_valid = RT_FALSE;

    result = rt_mutex_init(&g_cleaner.lock, "hclean_l", RT_IPC_FLAG_FIFO);
    if (result != RT_EOK)
    {
        LOG_E("cleaner mutex init failed result=%d", result);
        return result;
    }
    result = rt_sem_init(&g_cleaner.start_sem, "hclean_s", 0, RT_IPC_FLAG_FIFO);
    if (result != RT_EOK)
    {
        LOG_E("cleaner semaphore init failed result=%d", result);
        return result;
    }

    rt_pin_mode(g_cleaner.config.relay_pin, PIN_MODE_OUTPUT);
    rt_pin_write(g_cleaner.config.relay_pin, g_cleaner.config.relay_off_level);

    if (hcics_axis_init() != RT_EOK)
    {
        LOG_E("axis motor software TX unavailable");
        return -RT_ENOSYS;
    }

#if HCICS_CLEANER_HAS_PWM
    g_cleaner.pwm = (struct rt_device_pwm *)rt_device_find(g_cleaner.config.pwm_device_name);
    if (g_cleaner.pwm == RT_NULL)
    {
        LOG_E("PWM device %s not found",
              g_cleaner.config.pwm_device_name);
        return -RT_ENOSYS;
    }
#else
    LOG_E("RT_USING_PWM is disabled; cleaner servo output unavailable");
    return -RT_ENOSYS;
#endif

    g_cleaner.initialized = RT_TRUE;
    if (hcics_cleaner_safe_outputs() != RT_EOK)
    {
        g_cleaner.initialized = RT_FALSE;
        LOG_E("cleaner safe output check failed");
        return -RT_ERROR;
    }

    g_cleaner.worker = rt_thread_create("hclean",
                                        hcics_cleaner_worker,
                                        RT_NULL,
                                        HCICS_CLEANER_THREAD_STACK,
                                        HCICS_CLEANER_THREAD_PRIO,
                                        20);
    if (g_cleaner.worker == RT_NULL)
    {
        g_cleaner.initialized = RT_FALSE;
        LOG_E("worker thread create failed");
        return -RT_ENOMEM;
    }

    rt_thread_startup(g_cleaner.worker);
    LOG_I("cleaner initialized: pwm=%s y_ch=%d z_ch=%d relay=PB12",
          g_cleaner.config.pwm_device_name,
          g_cleaner.config.y_pwm_channel,
          g_cleaner.config.z_pwm_channel);

    return RT_EOK;
}

rt_err_t hcics_cleaner_update_camera_target(const hcics_cleaner_camera_target_t *target)
{
    hcics_cleaner_camera_target_t snapshot;

    if (target == RT_NULL)
    {
        return -RT_EINVAL;
    }
    if (!g_cleaner.initialized)
    {
        rt_err_t result = hcics_cleaner_init(RT_NULL);
        if (result != RT_EOK)
        {
            return result;
        }
    }

    snapshot = *target;
    if (snapshot.tick == 0U)
    {
        snapshot.tick = rt_tick_get();
    }

    rt_mutex_take(&g_cleaner.lock, RT_WAITING_FOREVER);
    g_cleaner.camera = snapshot;
    rt_mutex_release(&g_cleaner.lock);

    return RT_EOK;
}

rt_err_t hcics_cleaner_update_guard(const hcics_cleaner_guard_t *guard)
{
    hcics_cleaner_guard_t snapshot;

    if (guard == RT_NULL)
    {
        return -RT_EINVAL;
    }
    if (!g_cleaner.initialized)
    {
        rt_err_t result = hcics_cleaner_init(RT_NULL);
        if (result != RT_EOK)
        {
            return result;
        }
    }

    snapshot = *guard;
    if (snapshot.tick == 0U)
    {
        snapshot.tick = rt_tick_get();
    }
    if (snapshot.tof_mm == HCICS_CLEANER_TOF_INVALID_MM)
    {
        snapshot.tof_valid = RT_FALSE;
    }

    rt_mutex_take(&g_cleaner.lock, RT_WAITING_FOREVER);
    g_cleaner.guard = snapshot;
    if (snapshot.force_stop)
    {
        g_cleaner.stop_requested = RT_TRUE;
    }
    rt_mutex_release(&g_cleaner.lock);

    if (snapshot.force_stop)
    {
        (void)hcics_cleaner_safe_outputs();
    }

    return RT_EOK;
}

rt_err_t hcics_cleaner_start(rt_uint8_t max_cycles)
{
    rt_err_t result;

    if (!g_cleaner.initialized)
    {
        result = hcics_cleaner_init(RT_NULL);
        if (result != RT_EOK)
        {
            return result;
        }
    }

    rt_mutex_take(&g_cleaner.lock, RT_WAITING_FOREVER);
    if (g_cleaner.busy)
    {
        rt_mutex_release(&g_cleaner.lock);
        return -RT_EBUSY;
    }

    g_cleaner.busy = RT_TRUE;
    g_cleaner.stop_requested = RT_FALSE;
    g_cleaner.last_result = RT_EOK;
    g_cleaner.cycle = 0U;
    g_cleaner.max_cycles = (max_cycles == 0U) ? HCICS_CLEANER_MAX_CYCLES : max_cycles;
    g_cleaner.state = HCICS_CLEANER_STATE_AIMING;
    rt_mutex_release(&g_cleaner.lock);

    result = rt_sem_release(&g_cleaner.start_sem);
    if (result != RT_EOK)
    {
        rt_mutex_take(&g_cleaner.lock, RT_WAITING_FOREVER);
        g_cleaner.busy = RT_FALSE;
        g_cleaner.stop_requested = RT_TRUE;
        g_cleaner.last_result = result;
        g_cleaner.state = HCICS_CLEANER_STATE_ERROR;
        rt_mutex_release(&g_cleaner.lock);
        hcics_cleaner_relay(RT_FALSE);
        LOG_E("cleaner start semaphore release failed result=%d", result);
        return result;
    }

    return RT_EOK;
}

void hcics_cleaner_emergency_stop(void)
{
    if (!g_cleaner.initialized)
    {
        if (hcics_cleaner_init(RT_NULL) != RT_EOK)
        {
            return;
        }
    }

    rt_mutex_take(&g_cleaner.lock, RT_WAITING_FOREVER);
    g_cleaner.stop_requested = RT_TRUE;
    g_cleaner.state = HCICS_CLEANER_STATE_STOPPED;
    rt_mutex_release(&g_cleaner.lock);

    (void)hcics_cleaner_safe_outputs();
    hcics_axis_emergency_stop();
}

void hcics_cleaner_reset(void)
{
    rt_bool_t was_busy;

    if (!g_cleaner.initialized)
    {
        if (hcics_cleaner_init(RT_NULL) != RT_EOK)
        {
            return;
        }
    }

    rt_mutex_take(&g_cleaner.lock, RT_WAITING_FOREVER);
    was_busy = g_cleaner.busy;
    if (was_busy)
    {
        g_cleaner.stop_requested = RT_TRUE;
        g_cleaner.state = HCICS_CLEANER_STATE_STOPPED;
    }
    rt_mutex_release(&g_cleaner.lock);

    (void)hcics_cleaner_safe_outputs();
    (void)hcics_axis_stop();

    if (was_busy && !hcics_cleaner_wait_not_busy(HCICS_CLEANER_RESET_WAIT_MS))
    {
        LOG_E("cleaner reset wait busy timeout");
        return;
    }

    rt_mutex_take(&g_cleaner.lock, RT_WAITING_FOREVER);
    g_cleaner.busy = RT_FALSE;
    g_cleaner.stop_requested = RT_FALSE;
    g_cleaner.last_result = RT_EOK;
    g_cleaner.cycle = 0U;
    g_cleaner.state = HCICS_CLEANER_STATE_IDLE;
    rt_mutex_release(&g_cleaner.lock);
}

rt_err_t hcics_cleaner_get_status(hcics_cleaner_status_t *status)
{
    if (status == RT_NULL)
    {
        return -RT_EINVAL;
    }
    if (!g_cleaner.initialized)
    {
        rt_err_t result = hcics_cleaner_init(RT_NULL);
        if (result != RT_EOK)
        {
            return result;
        }
    }

    rt_mutex_take(&g_cleaner.lock, RT_WAITING_FOREVER);
    status->state = g_cleaner.state;
    status->initialized = g_cleaner.initialized;
    status->busy = g_cleaner.busy;
    status->stop_requested = g_cleaner.stop_requested;
    status->relay_on = g_cleaner.relay_on;
    status->last_result = g_cleaner.last_result;
    status->cycle = g_cleaner.cycle;
    status->max_cycles = g_cleaner.max_cycles;
    status->y_pulse_ns = g_cleaner.y_pulse_ns;
    status->z_pulse_ns = g_cleaner.z_pulse_ns;
    status->camera = g_cleaner.camera;
    status->guard = g_cleaner.guard;
    rt_mutex_release(&g_cleaner.lock);

    return RT_EOK;
}
