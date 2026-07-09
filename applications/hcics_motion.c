/*
 * HCICS chassis motion adapter.
 *
 * The chassis uses real PWM plus direction GPIO outputs on the ART-Pi P1/P2
 * headers.
 */

#include "hcics_motion.h"

#include <rtdevice.h>
#include "drv_common.h"
#include <stdlib.h>
#include <stdio.h>

#define DBG_TAG "hcics.motion"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#ifndef HCICS_MOTION_PWM_DEVICE_NAME
#define HCICS_MOTION_PWM_DEVICE_NAME       "pwm5"
#endif

#ifndef HCICS_MOTION_LEFT_PWM_CHANNEL
#define HCICS_MOTION_LEFT_PWM_CHANNEL      1
#endif

#ifndef HCICS_MOTION_RIGHT_PWM_CHANNEL
#define HCICS_MOTION_RIGHT_PWM_CHANNEL     2
#endif

#ifndef HCICS_MOTION_LEFT_ENCODER_NAME
#define HCICS_MOTION_LEFT_ENCODER_NAME     "pulse1"
#endif

#ifndef HCICS_MOTION_RIGHT_ENCODER_NAME
#define HCICS_MOTION_RIGHT_ENCODER_NAME    "pulse2"
#endif

#ifndef HCICS_MOTION_RAMP_STEP
#define HCICS_MOTION_RAMP_STEP             HCICS_MOTION_DEFAULT_RAMP_STEP
#endif

#ifndef HCICS_MOTION_MIN_NONZERO_CMD
#define HCICS_MOTION_MIN_NONZERO_CMD       25
#endif

#ifndef HCICS_MOTION_STUCK_MIN_CMD
#define HCICS_MOTION_STUCK_MIN_CMD         35
#endif

#ifndef HCICS_MOTION_STUCK_BOOST_CMD
#define HCICS_MOTION_STUCK_BOOST_CMD       90
#endif

#ifndef HCICS_MOTION_STUCK_ACTUAL_MAX
#define HCICS_MOTION_STUCK_ACTUAL_MAX      3
#endif

#ifndef HCICS_MOTION_STUCK_TRIGGER_MS
#define HCICS_MOTION_STUCK_TRIGGER_MS      220U
#endif

#ifndef HCICS_MOTION_STUCK_HOLD_MS
#define HCICS_MOTION_STUCK_HOLD_MS         450U
#endif

#ifndef HCICS_MOTION_STUCK_FAIL_MS
#define HCICS_MOTION_STUCK_FAIL_MS         1800U
#endif

#ifndef HCICS_MOTION_STUCK_MAX_BOOSTS
#define HCICS_MOTION_STUCK_MAX_BOOSTS      2U
#endif

#ifndef HCICS_MOTION_ENCODER_SANITY_LIMIT
#define HCICS_MOTION_ENCODER_SANITY_LIMIT  1200
#endif

#ifndef HCICS_MOTION_ENCODER_FAIL_LIMIT
#define HCICS_MOTION_ENCODER_FAIL_LIMIT    20U
#endif

#ifndef HCICS_MOTION_PWM_PERIOD_NS
#define HCICS_MOTION_PWM_PERIOD_NS         1000000U
#endif

#ifndef HCICS_MOTION_PWM_FULL_PULSE_NS
#define HCICS_MOTION_PWM_FULL_PULSE_NS     900000U
#endif

#ifndef HCICS_MOTION_LEFT_A_PIN
#define HCICS_MOTION_LEFT_A_PIN            GET_PIN(B, 1)
#endif

#ifndef HCICS_MOTION_LEFT_B_PIN
#define HCICS_MOTION_LEFT_B_PIN            GET_PIN(B, 2)
#endif

#ifndef HCICS_MOTION_RIGHT_A_PIN
#define HCICS_MOTION_RIGHT_A_PIN           GET_PIN(D, 13)
#endif

#ifndef HCICS_MOTION_RIGHT_B_PIN
#define HCICS_MOTION_RIGHT_B_PIN           GET_PIN(B, 0)
#endif

#ifndef HCICS_MOTION_STBY_PIN
#define HCICS_MOTION_STBY_PIN              GET_PIN(G, 7)
#endif

#ifndef HCICS_MOTION_STBY_ACTIVE_LEVEL
#define HCICS_MOTION_STBY_ACTIVE_LEVEL     PIN_HIGH
#endif

#if defined(RT_USING_PWM)
#include <drivers/rt_drv_pwm.h>
#define HCICS_MOTION_HAS_PWM               1
#else
#define HCICS_MOTION_HAS_PWM               0
#endif

#if defined(RT_USING_PULSE_ENCODER)
#include <drivers/pulse_encoder.h>
#define HCICS_MOTION_HAS_ENCODER           1
#else
#define HCICS_MOTION_HAS_ENCODER           0
#endif

typedef struct
{
    rt_bool_t initialized;
    rt_bool_t output_ready;
    rt_bool_t emergency_stop;
    rt_uint8_t stop_reason;
    hcics_motion_actual_source_t actual_source;

    rt_int16_t target_left;
    rt_int16_t target_right;
    rt_int16_t command_left;
    rt_int16_t command_right;
    rt_int16_t actual_left;
    rt_int16_t actual_right;
    rt_int16_t output_left;
    rt_int16_t output_right;

    rt_int32_t encoder_left_total;
    rt_int32_t encoder_right_total;
    rt_int32_t encoder_left_last;
    rt_int32_t encoder_right_last;
    rt_uint8_t encoder_left_fail_count;
    rt_uint8_t encoder_right_fail_count;

    rt_uint32_t stuck_start_ms;
    rt_uint32_t boost_until_ms;
    rt_uint8_t stuck_boost_count;
    rt_tick_t last_update_tick;

    rt_device_t left_encoder;
    rt_device_t right_encoder;
#if HCICS_MOTION_HAS_PWM
    struct rt_device_pwm *pwm;
#endif
    hcics_motion_output_cb_t output_cb;
    void *output_user_data;
} hcics_motion_context_t;

static hcics_motion_context_t g_motion;

static rt_uint32_t hcics_motion_now_ms(void)
{
    return (rt_uint32_t)(((rt_uint64_t)rt_tick_get() * 1000ULL) / RT_TICK_PER_SECOND);
}

static rt_int16_t hcics_motion_abs16(rt_int16_t value)
{
    return (value >= 0) ? value : (rt_int16_t)(-value);
}

static rt_int16_t hcics_motion_clamp_raw(rt_int16_t speed)
{
    if (speed > HCICS_MOTION_SPEED_MAX)
    {
        return HCICS_MOTION_SPEED_MAX;
    }
    if (speed < -HCICS_MOTION_SPEED_MAX)
    {
        return -HCICS_MOTION_SPEED_MAX;
    }

    return speed;
}

static rt_int16_t hcics_motion_normalize_target(rt_int16_t speed)
{
    rt_int16_t clamped = hcics_motion_clamp_raw(speed);
    rt_int16_t abs_speed = hcics_motion_abs16(clamped);

    if (abs_speed == 0)
    {
        return 0;
    }
    if (abs_speed < HCICS_MOTION_MIN_NONZERO_CMD)
    {
        return (clamped > 0) ? HCICS_MOTION_MIN_NONZERO_CMD :
                               (rt_int16_t)(-HCICS_MOTION_MIN_NONZERO_CMD);
    }

    return clamped;
}

static rt_int16_t hcics_motion_ramp_toward(rt_int16_t current, rt_int16_t target)
{
    if (target > current)
    {
        current = (rt_int16_t)(current + HCICS_MOTION_RAMP_STEP);
        if (current > target)
        {
            current = target;
        }
    }
    else if (target < current)
    {
        current = (rt_int16_t)(current - HCICS_MOTION_RAMP_STEP);
        if (current < target)
        {
            current = target;
        }
    }

    return current;
}

static rt_int16_t hcics_motion_preserve_direction(rt_int16_t speed, rt_int16_t min_abs)
{
    rt_int16_t abs_speed = hcics_motion_abs16(speed);

    if ((speed == 0) || (abs_speed >= min_abs))
    {
        return speed;
    }

    return (speed > 0) ? min_abs : (rt_int16_t)(-min_abs);
}

static void hcics_motion_refresh_output_mode(void)
{
    g_motion.output_ready = RT_FALSE;
#if HCICS_MOTION_HAS_PWM
    if (g_motion.pwm != RT_NULL)
    {
        g_motion.output_ready = RT_TRUE;
    }
#endif
}

static rt_base_t hcics_motion_inactive_stby_level(void)
{
    return (HCICS_MOTION_STBY_ACTIVE_LEVEL == PIN_HIGH) ? PIN_LOW : PIN_HIGH;
}

static void hcics_motion_gpio_safe(void)
{
    rt_pin_write(HCICS_MOTION_LEFT_A_PIN, PIN_LOW);
    rt_pin_write(HCICS_MOTION_LEFT_B_PIN, PIN_LOW);
    rt_pin_write(HCICS_MOTION_RIGHT_A_PIN, PIN_LOW);
    rt_pin_write(HCICS_MOTION_RIGHT_B_PIN, PIN_LOW);
    rt_pin_write(HCICS_MOTION_STBY_PIN, hcics_motion_inactive_stby_level());
}

static void hcics_motion_gpio_init(void)
{
    rt_pin_mode(HCICS_MOTION_LEFT_A_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(HCICS_MOTION_LEFT_B_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(HCICS_MOTION_RIGHT_A_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(HCICS_MOTION_RIGHT_B_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(HCICS_MOTION_STBY_PIN, PIN_MODE_OUTPUT);
    hcics_motion_gpio_safe();
}

static void hcics_motion_write_dir(rt_base_t pin_a,
                                   rt_base_t pin_b,
                                   rt_int16_t speed)
{
    if (speed > 0)
    {
        rt_pin_write(pin_a, PIN_HIGH);
        rt_pin_write(pin_b, PIN_LOW);
        return;
    }
    if (speed < 0)
    {
        rt_pin_write(pin_a, PIN_LOW);
        rt_pin_write(pin_b, PIN_HIGH);
        return;
    }

    rt_pin_write(pin_a, PIN_LOW);
    rt_pin_write(pin_b, PIN_LOW);
}

#if HCICS_MOTION_HAS_PWM
static void hcics_motion_find_pwm(void)
{
    g_motion.pwm = (struct rt_device_pwm *)rt_device_find(HCICS_MOTION_PWM_DEVICE_NAME);
    if (g_motion.pwm == RT_NULL)
    {
        LOG_W("PWM device %s not found; PWM motor output disabled",
              HCICS_MOTION_PWM_DEVICE_NAME);
        return;
    }

    LOG_I("PWM chassis output ready pwm=%s left_ch=%d right_ch=%d",
          HCICS_MOTION_PWM_DEVICE_NAME,
          HCICS_MOTION_LEFT_PWM_CHANNEL,
          HCICS_MOTION_RIGHT_PWM_CHANNEL);
}
#else
static void hcics_motion_find_pwm(void)
{
    LOG_E("RT_USING_PWM is disabled; PWM motor output unavailable");
}
#endif

#if HCICS_MOTION_HAS_ENCODER
static rt_device_t hcics_motion_open_encoder(const char *name, rt_int32_t *last_count)
{
    rt_device_t dev = rt_device_find(name);
    rt_int32_t count = 0;

    if (dev == RT_NULL)
    {
        LOG_W("encoder %s not found; actual speed feedback unavailable", name);
        return RT_NULL;
    }

    if (rt_device_open(dev, RT_DEVICE_FLAG_RDONLY) != RT_EOK)
    {
        LOG_W("encoder %s open failed; actual speed feedback unavailable", name);
        return RT_NULL;
    }

    (void)rt_device_control(dev, PULSE_ENCODER_CMD_CLEAR_COUNT, RT_NULL);
    if (rt_device_read(dev, 0, &count, 1) == 1)
    {
        *last_count = count;
    }
    else
    {
        *last_count = 0;
    }

    LOG_I("encoder %s ready", name);
    return dev;
}
#else
static rt_device_t hcics_motion_open_encoder(const char *name, rt_int32_t *last_count)
{
    (void)name;
    *last_count = 0;
    LOG_W("RT_USING_PULSE_ENCODER is disabled; actual speed feedback unavailable");
    return RT_NULL;
}
#endif

static rt_err_t hcics_motion_ensure_init(void)
{
    if (!g_motion.initialized)
    {
        return hcics_motion_init();
    }

    return RT_EOK;
}

#if HCICS_MOTION_HAS_PWM
static rt_err_t hcics_motion_pwm_fail(rt_err_t result, const char *stage)
{
    (void)rt_pwm_disable(g_motion.pwm, HCICS_MOTION_LEFT_PWM_CHANNEL);
    (void)rt_pwm_disable(g_motion.pwm, HCICS_MOTION_RIGHT_PWM_CHANNEL);
    hcics_motion_gpio_safe();
    g_motion.emergency_stop = RT_TRUE;
    g_motion.stop_reason = 0xE1U;
    LOG_E("PWM chassis output failed stage=%s result=%d", stage, result);
    return result;
}

static rt_err_t hcics_motion_output_pwm(rt_int16_t left, rt_int16_t right)
{
    rt_uint32_t left_pulse;
    rt_uint32_t right_pulse;
    rt_err_t result;

    if (g_motion.pwm == RT_NULL)
    {
        hcics_motion_gpio_safe();
        return -RT_ENOSYS;
    }

    hcics_motion_write_dir(HCICS_MOTION_LEFT_A_PIN,
                           HCICS_MOTION_LEFT_B_PIN,
                           left);
    hcics_motion_write_dir(HCICS_MOTION_RIGHT_A_PIN,
                           HCICS_MOTION_RIGHT_B_PIN,
                           right);
    rt_pin_write(HCICS_MOTION_STBY_PIN,
                 ((left != 0) || (right != 0)) ?
                 HCICS_MOTION_STBY_ACTIVE_LEVEL :
                 hcics_motion_inactive_stby_level());

    left_pulse = ((rt_uint32_t)hcics_motion_abs16(left) * HCICS_MOTION_PWM_FULL_PULSE_NS) /
                 HCICS_MOTION_SPEED_MAX;
    right_pulse = ((rt_uint32_t)hcics_motion_abs16(right) * HCICS_MOTION_PWM_FULL_PULSE_NS) /
                  HCICS_MOTION_SPEED_MAX;

    result = rt_pwm_set(g_motion.pwm, HCICS_MOTION_LEFT_PWM_CHANNEL,
                        HCICS_MOTION_PWM_PERIOD_NS, left_pulse);
    if (result != RT_EOK)
    {
        return hcics_motion_pwm_fail(result, "left-set");
    }
    result = rt_pwm_enable(g_motion.pwm, HCICS_MOTION_LEFT_PWM_CHANNEL);
    if (result != RT_EOK)
    {
        return hcics_motion_pwm_fail(result, "left-enable");
    }
    result = rt_pwm_set(g_motion.pwm, HCICS_MOTION_RIGHT_PWM_CHANNEL,
                        HCICS_MOTION_PWM_PERIOD_NS, right_pulse);
    if (result != RT_EOK)
    {
        return hcics_motion_pwm_fail(result, "right-set");
    }
    result = rt_pwm_enable(g_motion.pwm, HCICS_MOTION_RIGHT_PWM_CHANNEL);
    if (result != RT_EOK)
    {
        return hcics_motion_pwm_fail(result, "right-enable");
    }

    return RT_EOK;
}
#else
static rt_err_t hcics_motion_output_pwm(rt_int16_t left, rt_int16_t right)
{
    (void)left;
    (void)right;
    hcics_motion_gpio_safe();
    return -RT_ENOSYS;
}
#endif

static rt_err_t hcics_motion_output(rt_int16_t left, rt_int16_t right)
{
    rt_err_t result = RT_EOK;

    g_motion.output_left = left;
    g_motion.output_right = right;

    if (g_motion.output_cb != RT_NULL)
    {
        result = g_motion.output_cb(left, right, g_motion.output_user_data);
    }

    if (hcics_motion_output_pwm(left, right) != RT_EOK)
    {
        result = -RT_ERROR;
    }

    return result;
}

static rt_bool_t hcics_motion_read_encoder_delta(rt_device_t dev,
                                                 rt_int32_t *last_count,
                                                 rt_int16_t *delta)
{
    rt_int32_t count = 0;
    rt_int32_t diff;

    if ((dev == RT_NULL) || (last_count == RT_NULL) || (delta == RT_NULL))
    {
        return RT_FALSE;
    }

    if (rt_device_read(dev, 0, &count, 1) != 1)
    {
        return RT_FALSE;
    }

    diff = count - *last_count;
    *last_count = count;

    if ((diff > HCICS_MOTION_ENCODER_SANITY_LIMIT) ||
        (diff < -HCICS_MOTION_ENCODER_SANITY_LIMIT))
    {
        *last_count = count;
        *delta = 0;
        return RT_FALSE;
    }

    *delta = (rt_int16_t)diff;
    return RT_TRUE;
}

static rt_err_t hcics_motion_poll_actual_speed(void)
{
    rt_int16_t left_delta = 0;
    rt_int16_t right_delta = 0;
    rt_bool_t left_ok;
    rt_bool_t right_ok;
    rt_bool_t motion_requested;

    left_ok = hcics_motion_read_encoder_delta(g_motion.left_encoder,
                                              &g_motion.encoder_left_last,
                                              &left_delta);
    right_ok = hcics_motion_read_encoder_delta(g_motion.right_encoder,
                                               &g_motion.encoder_right_last,
                                               &right_delta);
    motion_requested = ((g_motion.target_left != 0) ||
                        (g_motion.target_right != 0) ||
                        (g_motion.command_left != 0) ||
                        (g_motion.command_right != 0)) ? RT_TRUE : RT_FALSE;

    if (left_ok)
    {
        g_motion.encoder_left_fail_count = 0U;
    }
    else if (motion_requested && (g_motion.encoder_left_fail_count < 255U))
    {
        g_motion.encoder_left_fail_count++;
    }

    if (right_ok)
    {
        g_motion.encoder_right_fail_count = 0U;
    }
    else if (motion_requested && (g_motion.encoder_right_fail_count < 255U))
    {
        g_motion.encoder_right_fail_count++;
    }

    if (motion_requested &&
        ((g_motion.encoder_left_fail_count >= HCICS_MOTION_ENCODER_FAIL_LIMIT) ||
         (g_motion.encoder_right_fail_count >= HCICS_MOTION_ENCODER_FAIL_LIMIT)))
    {
        g_motion.emergency_stop = RT_TRUE;
        g_motion.stop_reason = 0xE2U;
        g_motion.target_left = 0;
        g_motion.target_right = 0;
        g_motion.command_left = 0;
        g_motion.command_right = 0;
        LOG_E("encoder read failed left_fail=%u right_fail=%u",
              g_motion.encoder_left_fail_count,
              g_motion.encoder_right_fail_count);
        return -RT_ERROR;
    }

    if (left_ok || right_ok)
    {
        g_motion.actual_left = left_ok ? left_delta : 0;
        g_motion.actual_right = right_ok ? right_delta : 0;
        g_motion.encoder_left_total += g_motion.actual_left;
        g_motion.encoder_right_total += g_motion.actual_right;
        g_motion.actual_source = HCICS_MOTION_ACTUAL_ENCODER;
    }

    return RT_EOK;
}

static rt_err_t hcics_motion_apply_stuck_boost(rt_int16_t *left, rt_int16_t *right)
{
    rt_uint32_t now_ms;
    rt_bool_t command_nonzero;
    rt_bool_t actual_stalled;

    if ((left == RT_NULL) || (right == RT_NULL))
    {
        return -RT_EINVAL;
    }

    if (g_motion.actual_source == HCICS_MOTION_ACTUAL_NONE)
    {
        g_motion.stuck_start_ms = 0U;
        g_motion.boost_until_ms = 0U;
        g_motion.stuck_boost_count = 0U;
        return RT_EOK;
    }

    now_ms = hcics_motion_now_ms();
    command_nonzero = ((hcics_motion_abs16(*left) >= HCICS_MOTION_STUCK_MIN_CMD) ||
                       (hcics_motion_abs16(*right) >= HCICS_MOTION_STUCK_MIN_CMD)) ?
                      RT_TRUE : RT_FALSE;
    actual_stalled = ((hcics_motion_abs16(g_motion.actual_left) <= HCICS_MOTION_STUCK_ACTUAL_MAX) &&
                      (hcics_motion_abs16(g_motion.actual_right) <= HCICS_MOTION_STUCK_ACTUAL_MAX)) ?
                     RT_TRUE : RT_FALSE;

    if (command_nonzero && actual_stalled)
    {
        if (g_motion.stuck_start_ms == 0U)
        {
            g_motion.stuck_start_ms = now_ms;
        }
        else if (((now_ms - g_motion.stuck_start_ms) >= HCICS_MOTION_STUCK_TRIGGER_MS) &&
                 (g_motion.boost_until_ms == 0U))
        {
            if (g_motion.stuck_boost_count >= HCICS_MOTION_STUCK_MAX_BOOSTS)
            {
                g_motion.emergency_stop = RT_TRUE;
                g_motion.stop_reason = 0xE3U;
                g_motion.target_left = 0;
                g_motion.target_right = 0;
                g_motion.command_left = 0;
                g_motion.command_right = 0;
                g_motion.boost_until_ms = 0U;
                LOG_E("stuck fault cmd=%d,%d actual=%d,%d boosts=%u",
                      (int)*left,
                      (int)*right,
                      (int)g_motion.actual_left,
                      (int)g_motion.actual_right,
                      (unsigned int)g_motion.stuck_boost_count);
                return -RT_ERROR;
            }

            g_motion.boost_until_ms = now_ms + HCICS_MOTION_STUCK_HOLD_MS;
            g_motion.stuck_boost_count++;
            LOG_W("stuck boost start cmd=%d,%d actual=%d,%d",
                  (int)*left,
                  (int)*right,
                  (int)g_motion.actual_left,
                  (int)g_motion.actual_right);
        }
    }
    else
    {
        g_motion.stuck_start_ms = 0U;
        g_motion.stuck_boost_count = 0U;
    }

    if (command_nonzero && actual_stalled &&
        ((now_ms - g_motion.stuck_start_ms) >= HCICS_MOTION_STUCK_FAIL_MS))
    {
        g_motion.emergency_stop = RT_TRUE;
        g_motion.stop_reason = 0xE3U;
        g_motion.target_left = 0;
        g_motion.target_right = 0;
        g_motion.command_left = 0;
        g_motion.command_right = 0;
        g_motion.boost_until_ms = 0U;
        LOG_E("stuck timeout cmd=%d,%d actual=%d,%d",
              (int)*left,
              (int)*right,
              (int)g_motion.actual_left,
              (int)g_motion.actual_right);
        return -RT_ERROR;
    }

    if ((g_motion.boost_until_ms != 0U) && (now_ms < g_motion.boost_until_ms))
    {
        *left = hcics_motion_preserve_direction(*left, HCICS_MOTION_STUCK_BOOST_CMD);
        *right = hcics_motion_preserve_direction(*right, HCICS_MOTION_STUCK_BOOST_CMD);
        return RT_EOK;
    }

    if ((g_motion.boost_until_ms != 0U) && (now_ms >= g_motion.boost_until_ms))
    {
        g_motion.boost_until_ms = 0U;
        g_motion.stuck_start_ms = 0U;
        LOG_I("stuck boost end actual=%d,%d",
              (int)g_motion.actual_left,
              (int)g_motion.actual_right);
    }

    return RT_EOK;
}

static rt_err_t hcics_motion_service(rt_bool_t direct)
{
    rt_int16_t out_left;
    rt_int16_t out_right;
    rt_err_t result;

    if (hcics_motion_ensure_init() != RT_EOK)
    {
        return -RT_ERROR;
    }

    result = hcics_motion_poll_actual_speed();
    if (result != RT_EOK)
    {
        return hcics_motion_output(0, 0);
    }

    if (g_motion.emergency_stop)
    {
        g_motion.target_left = 0;
        g_motion.target_right = 0;
        g_motion.command_left = 0;
        g_motion.command_right = 0;
        g_motion.stuck_start_ms = 0U;
        g_motion.boost_until_ms = 0U;
        g_motion.last_update_tick = rt_tick_get();
        return hcics_motion_output(0, 0);
    }

    if (direct)
    {
        g_motion.command_left = g_motion.target_left;
        g_motion.command_right = g_motion.target_right;
        g_motion.stuck_start_ms = 0U;
        g_motion.boost_until_ms = 0U;
    }
    else
    {
        g_motion.command_left = hcics_motion_ramp_toward(g_motion.command_left,
                                                         g_motion.target_left);
        g_motion.command_right = hcics_motion_ramp_toward(g_motion.command_right,
                                                          g_motion.target_right);
    }

    out_left = hcics_motion_clamp_raw(g_motion.command_left);
    out_right = hcics_motion_clamp_raw(g_motion.command_right);
    if (!direct)
    {
        result = hcics_motion_apply_stuck_boost(&out_left, &out_right);
        if (result != RT_EOK)
        {
            return hcics_motion_output(0, 0);
        }
    }

    g_motion.last_update_tick = rt_tick_get();
    return hcics_motion_output(hcics_motion_clamp_raw(out_left),
                               hcics_motion_clamp_raw(out_right));
}

rt_err_t hcics_motion_init(void)
{
    if (g_motion.initialized)
    {
        return RT_EOK;
    }

    g_motion.target_left = 0;
    g_motion.target_right = 0;
    g_motion.command_left = 0;
    g_motion.command_right = 0;
    g_motion.actual_left = 0;
    g_motion.actual_right = 0;
    g_motion.actual_source = HCICS_MOTION_ACTUAL_NONE;

    hcics_motion_gpio_init();
    hcics_motion_find_pwm();
    g_motion.left_encoder = hcics_motion_open_encoder(HCICS_MOTION_LEFT_ENCODER_NAME,
                                                      &g_motion.encoder_left_last);
    g_motion.right_encoder = hcics_motion_open_encoder(HCICS_MOTION_RIGHT_ENCODER_NAME,
                                                       &g_motion.encoder_right_last);
    if ((g_motion.left_encoder == RT_NULL) || (g_motion.right_encoder == RT_NULL))
    {
        LOG_E("required chassis encoders not ready left=%p right=%p",
              g_motion.left_encoder, g_motion.right_encoder);
        return -RT_ENOSYS;
    }

    hcics_motion_refresh_output_mode();

    if (!g_motion.output_ready)
    {
        LOG_E("no motor output device configured");
        return -RT_ENOSYS;
    }

    g_motion.initialized = RT_TRUE;
    LOG_I("motion adapter initialized output_ready=%u",
          (unsigned int)g_motion.output_ready);
    (void)hcics_motion_output(0, 0);
    return RT_EOK;
}

rt_err_t hcics_motion_set_output_callback(hcics_motion_output_cb_t callback,
                                          void *user_data)
{
    if (hcics_motion_ensure_init() != RT_EOK)
    {
        return -RT_ERROR;
    }

    g_motion.output_cb = callback;
    g_motion.output_user_data = user_data;
    hcics_motion_refresh_output_mode();
    return RT_EOK;
}

rt_err_t hcics_motion_set_target(rt_int16_t left, rt_int16_t right)
{
    if (hcics_motion_ensure_init() != RT_EOK)
    {
        return -RT_ERROR;
    }

    if (g_motion.emergency_stop)
    {
        LOG_W("target ignored during emergency stop");
        return -RT_ERROR;
    }

    g_motion.target_left = hcics_motion_normalize_target(left);
    g_motion.target_right = hcics_motion_normalize_target(right);
    return RT_EOK;
}

rt_err_t hcics_motion_set_target_direct(rt_int16_t left, rt_int16_t right)
{
    if (hcics_motion_ensure_init() != RT_EOK)
    {
        return -RT_ERROR;
    }

    if (g_motion.emergency_stop)
    {
        LOG_W("direct target ignored during emergency stop");
        return -RT_ERROR;
    }

    g_motion.target_left = hcics_motion_normalize_target(left);
    g_motion.target_right = hcics_motion_normalize_target(right);
    return hcics_motion_service(RT_TRUE);
}

rt_err_t hcics_motion_update(void)
{
    return hcics_motion_service(RT_FALSE);
}

void hcics_motion_emergency_stop(rt_uint8_t reason)
{
    if (hcics_motion_ensure_init() != RT_EOK)
    {
        return;
    }

    g_motion.emergency_stop = RT_TRUE;
    g_motion.stop_reason = reason;
    g_motion.target_left = 0;
    g_motion.target_right = 0;
    g_motion.command_left = 0;
    g_motion.command_right = 0;
    g_motion.stuck_start_ms = 0U;
    g_motion.boost_until_ms = 0U;
    LOG_W("emergency stop reason=%u", (unsigned int)reason);
    (void)hcics_motion_output(0, 0);
}

void hcics_motion_clear_emergency_stop(void)
{
    if (hcics_motion_ensure_init() != RT_EOK)
    {
        return;
    }

    g_motion.emergency_stop = RT_FALSE;
    g_motion.stop_reason = 0U;
    LOG_I("emergency stop cleared");
}

void hcics_motion_reset(void)
{
    if (hcics_motion_ensure_init() != RT_EOK)
    {
        return;
    }

    g_motion.target_left = 0;
    g_motion.target_right = 0;
    g_motion.command_left = 0;
    g_motion.command_right = 0;
    g_motion.actual_left = 0;
    g_motion.actual_right = 0;
    g_motion.output_left = 0;
    g_motion.output_right = 0;
    g_motion.encoder_left_total = 0;
    g_motion.encoder_right_total = 0;
    g_motion.encoder_left_fail_count = 0U;
    g_motion.encoder_right_fail_count = 0U;
    g_motion.stuck_start_ms = 0U;
    g_motion.boost_until_ms = 0U;
    g_motion.stuck_boost_count = 0U;
    g_motion.actual_source = HCICS_MOTION_ACTUAL_NONE;
    (void)hcics_motion_output(0, 0);
}

rt_err_t hcics_motion_set_actual_speed(rt_int16_t left, rt_int16_t right)
{
    if (hcics_motion_ensure_init() != RT_EOK)
    {
        return -RT_ERROR;
    }

    g_motion.actual_left = left;
    g_motion.actual_right = right;
    g_motion.actual_source = HCICS_MOTION_ACTUAL_EXTERNAL;
    return RT_EOK;
}

rt_err_t hcics_motion_set_encoder_delta(rt_int16_t left_delta,
                                        rt_int16_t right_delta)
{
    if (hcics_motion_ensure_init() != RT_EOK)
    {
        return -RT_ERROR;
    }

    g_motion.actual_left = left_delta;
    g_motion.actual_right = right_delta;
    g_motion.encoder_left_total += left_delta;
    g_motion.encoder_right_total += right_delta;
    g_motion.actual_source = HCICS_MOTION_ACTUAL_EXTERNAL;
    return RT_EOK;
}

void hcics_motion_get_realtime(rt_int16_t *target_left,
                               rt_int16_t *actual_left,
                               rt_int16_t *target_right,
                               rt_int16_t *actual_right)
{
    if (target_left != RT_NULL)
    {
        *target_left = g_motion.target_left;
    }
    if (actual_left != RT_NULL)
    {
        *actual_left = g_motion.actual_left;
    }
    if (target_right != RT_NULL)
    {
        *target_right = g_motion.target_right;
    }
    if (actual_right != RT_NULL)
    {
        *actual_right = g_motion.actual_right;
    }
}

void hcics_motion_get_snapshot(hcics_motion_snapshot_t *snapshot)
{
    rt_uint32_t now_ms;

    if (snapshot == RT_NULL)
    {
        return;
    }

    now_ms = hcics_motion_now_ms();
    snapshot->initialized = g_motion.initialized;
    snapshot->output_ready = g_motion.output_ready;
    snapshot->emergency_stop = g_motion.emergency_stop;
    snapshot->stop_reason = g_motion.stop_reason;
    snapshot->actual_source = g_motion.actual_source;
    snapshot->target_left = g_motion.target_left;
    snapshot->target_right = g_motion.target_right;
    snapshot->command_left = g_motion.command_left;
    snapshot->command_right = g_motion.command_right;
    snapshot->actual_left = g_motion.actual_left;
    snapshot->actual_right = g_motion.actual_right;
    snapshot->output_left = g_motion.output_left;
    snapshot->output_right = g_motion.output_right;
    snapshot->encoder_left_total = g_motion.encoder_left_total;
    snapshot->encoder_right_total = g_motion.encoder_right_total;
    if ((g_motion.boost_until_ms != 0U) &&
        ((rt_int32_t)(g_motion.boost_until_ms - now_ms) > 0))
    {
        snapshot->boost_active_ms = g_motion.boost_until_ms - now_ms;
    }
    else
    {
        snapshot->boost_active_ms = 0U;
    }
    snapshot->last_update_tick = g_motion.last_update_tick;
}

static const char *hcics_motion_actual_source_name(hcics_motion_actual_source_t source)
{
    switch (source)
    {
    case HCICS_MOTION_ACTUAL_EXTERNAL:
        return "external";
    case HCICS_MOTION_ACTUAL_ENCODER:
        return "encoder";
    case HCICS_MOTION_ACTUAL_NONE:
    default:
        return "none";
    }
}

void hcics_motion_status_dump(void)
{
    hcics_motion_snapshot_t snap;

    (void)hcics_motion_ensure_init();
    hcics_motion_get_snapshot(&snap);

    rt_kprintf("motion init=%u ready=%u estop=%u reason=%u src=%s\n",
               (unsigned int)snap.initialized,
               (unsigned int)snap.output_ready,
               (unsigned int)snap.emergency_stop,
               (unsigned int)snap.stop_reason,
               hcics_motion_actual_source_name(snap.actual_source));
    rt_kprintf("target=%d,%d cmd=%d,%d actual=%d,%d out=%d,%d\n",
               (int)snap.target_left,
               (int)snap.target_right,
               (int)snap.command_left,
               (int)snap.command_right,
               (int)snap.actual_left,
               (int)snap.actual_right,
               (int)snap.output_left,
               (int)snap.output_right);
    rt_kprintf("encoder_total=%ld,%ld boost_left_ms=%lu tick=%lu\n",
               (long)snap.encoder_left_total,
               (long)snap.encoder_right_total,
               (unsigned long)snap.boost_active_ms,
               (unsigned long)snap.last_update_tick);
}
