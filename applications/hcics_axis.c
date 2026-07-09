/*
 * HCICS X-axis motor driver.
 *
 * The wire protocol follows the ZDT/Emm42-style UART frames used by the
 * original vehicle project.  The ART-Pi P1/P2 wiring profile sends the
 * command stream on a software UART TX pin so both hardware UART pairs can
 * be used by the two cameras.
 */

#include "hcics_axis.h"

#include <rtdevice.h>
#include <rthw.h>
#include "drv_common.h"

#define DBG_TAG "hcics.axis"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#ifndef HCICS_AXIS_SOFT_TX_PIN
#define HCICS_AXIS_SOFT_TX_PIN           GET_PIN(A, 8)
#endif

#ifndef HCICS_AXIS_SOFT_TX_PIN_NAME
#define HCICS_AXIS_SOFT_TX_PIN_NAME      "PA8"
#endif

#ifndef HCICS_AXIS_SOFT_BAUD
#define HCICS_AXIS_SOFT_BAUD             115200U
#endif

#ifndef HCICS_AXIS_MOTOR_ID
#define HCICS_AXIS_MOTOR_ID              1U
#endif

#ifndef HCICS_AXIS_SPEED
#define HCICS_AXIS_SPEED                 180U
#endif

#ifndef HCICS_AXIS_SWEEP_PULSES
#define HCICS_AXIS_SWEEP_PULSES          700U
#endif

#ifndef HCICS_AXIS_PULSES_PER_PIXEL
#define HCICS_AXIS_PULSES_PER_PIXEL      6U
#endif

#ifndef HCICS_AXIS_CENTER_DEADBAND_PX
#define HCICS_AXIS_CENTER_DEADBAND_PX    18
#endif

#ifndef HCICS_AXIS_CENTER_MAX_PULSES
#define HCICS_AXIS_CENTER_MAX_PULSES     1200U
#endif

#ifndef HCICS_AXIS_DIR_POSITIVE
#define HCICS_AXIS_DIR_POSITIVE          0x01U
#endif

#ifndef HCICS_AXIS_DIR_NEGATIVE
#define HCICS_AXIS_DIR_NEGATIVE          0x00U
#endif

#ifndef HCICS_AXIS_INVERT_DIRECTION
#define HCICS_AXIS_INVERT_DIRECTION      0
#endif

typedef struct
{
    rt_bool_t initialized;
    rt_bool_t ready;
    rt_base_t tx_pin;
    struct rt_mutex lock;
} hcics_axis_context_t;

static hcics_axis_context_t g_axis;

static rt_uint8_t hcics_axis_dir(rt_bool_t positive)
{
    rt_bool_t actual_positive = positive;

#if HCICS_AXIS_INVERT_DIRECTION
    actual_positive = positive ? RT_FALSE : RT_TRUE;
#endif

    return actual_positive ? HCICS_AXIS_DIR_POSITIVE : HCICS_AXIS_DIR_NEGATIVE;
}

static rt_uint16_t hcics_axis_clamp_speed(rt_uint16_t speed)
{
    if (speed == 0U)
    {
        return 0U;
    }
    if (speed > 3000U)
    {
        return 3000U;
    }
    return speed;
}

static void hcics_axis_soft_delay_bit(rt_uint32_t *acc)
{
    rt_uint32_t delay_us;

    *acc += 1000000U;
    delay_us = *acc / HCICS_AXIS_SOFT_BAUD;
    *acc -= delay_us * HCICS_AXIS_SOFT_BAUD;
    if (delay_us == 0U)
    {
        delay_us = 1U;
    }

    rt_hw_us_delay(delay_us);
}

static void hcics_axis_soft_write_byte(rt_uint8_t byte)
{
    rt_uint8_t bit;
    rt_uint32_t acc = 0U;

    rt_pin_write(g_axis.tx_pin, PIN_LOW);
    hcics_axis_soft_delay_bit(&acc);

    for (bit = 0U; bit < 8U; bit++)
    {
        rt_pin_write(g_axis.tx_pin,
                     (byte & 0x01U) ? PIN_HIGH : PIN_LOW);
        hcics_axis_soft_delay_bit(&acc);
        byte >>= 1;
    }

    rt_pin_write(g_axis.tx_pin, PIN_HIGH);
    hcics_axis_soft_delay_bit(&acc);
}

static rt_err_t hcics_axis_write_locked(const rt_uint8_t *frame, rt_size_t len)
{
    rt_size_t i;
    rt_base_t level;

    if ((frame == RT_NULL) || (len == 0U))
    {
        return -RT_EINVAL;
    }

    level = rt_hw_interrupt_disable();
    for (i = 0U; i < len; i++)
    {
        hcics_axis_soft_write_byte(frame[i]);
    }
    rt_hw_interrupt_enable(level);

    return RT_EOK;
}

static rt_err_t hcics_axis_write(const rt_uint8_t *frame, rt_size_t len)
{
    rt_err_t result;

    if (!g_axis.ready)
    {
        return -RT_ENOSYS;
    }

    rt_mutex_take(&g_axis.lock, RT_WAITING_FOREVER);
    result = hcics_axis_write_locked(frame, len);
    rt_mutex_release(&g_axis.lock);

    return result;
}

rt_err_t hcics_axis_enable(void)
{
    rt_uint8_t frame[4];

    if (!g_axis.ready)
    {
        return -RT_ENOSYS;
    }

    frame[0] = HCICS_AXIS_MOTOR_ID;
    frame[1] = 0xF3U;
    frame[2] = 0xABU;
    frame[3] = (rt_uint8_t)(frame[0] + frame[1] + frame[2]);

    return hcics_axis_write(frame, sizeof(frame));
}

rt_err_t hcics_axis_speed(rt_bool_t positive, rt_uint16_t speed)
{
    rt_uint8_t frame[8];
    rt_uint16_t clamped = hcics_axis_clamp_speed(speed);

    frame[0] = HCICS_AXIS_MOTOR_ID;
    frame[1] = 0xF6U;
    frame[2] = hcics_axis_dir(positive);
    frame[3] = (rt_uint8_t)(clamped >> 8);
    frame[4] = (rt_uint8_t)(clamped & 0xFFU);
    frame[5] = 0x00U;
    frame[6] = 0x00U;
    frame[7] = 0x6BU;

    return hcics_axis_write(frame, sizeof(frame));
}

rt_err_t hcics_axis_stop(void)
{
    return hcics_axis_speed(RT_TRUE, 0U);
}

rt_err_t hcics_axis_move_pulses(rt_bool_t positive,
                                rt_uint16_t speed,
                                rt_uint32_t pulses)
{
    rt_uint8_t frame[13];
    rt_uint16_t clamped = hcics_axis_clamp_speed(speed);

    if (pulses == 0U)
    {
        return hcics_axis_stop();
    }

    frame[0] = HCICS_AXIS_MOTOR_ID;
    frame[1] = 0xFDU;
    frame[2] = hcics_axis_dir(positive);
    frame[3] = (rt_uint8_t)(clamped >> 8);
    frame[4] = (rt_uint8_t)(clamped & 0xFFU);
    frame[5] = 0x00U;
    frame[6] = (rt_uint8_t)((pulses >> 24) & 0xFFU);
    frame[7] = (rt_uint8_t)((pulses >> 16) & 0xFFU);
    frame[8] = (rt_uint8_t)((pulses >> 8) & 0xFFU);
    frame[9] = (rt_uint8_t)(pulses & 0xFFU);
    frame[10] = 0x00U;
    frame[11] = 0x00U;
    frame[12] = 0x6BU;

    return hcics_axis_write(frame, sizeof(frame));
}

rt_err_t hcics_axis_center_from_error(int error_x)
{
    rt_bool_t positive;
    rt_uint32_t pulses;
    int abs_error = error_x;

    if (abs_error < 0)
    {
        abs_error = -abs_error;
    }
    if (abs_error <= HCICS_AXIS_CENTER_DEADBAND_PX)
    {
        return hcics_axis_stop();
    }

    positive = (error_x > 0) ? RT_TRUE : RT_FALSE;
    pulses = (rt_uint32_t)abs_error * HCICS_AXIS_PULSES_PER_PIXEL;
    if (pulses > HCICS_AXIS_CENTER_MAX_PULSES)
    {
        pulses = HCICS_AXIS_CENTER_MAX_PULSES;
    }

    return hcics_axis_move_pulses(positive, HCICS_AXIS_SPEED, pulses);
}

rt_err_t hcics_axis_sweep_step(rt_bool_t positive)
{
    return hcics_axis_move_pulses(positive,
                                  HCICS_AXIS_SPEED,
                                  HCICS_AXIS_SWEEP_PULSES);
}

void hcics_axis_emergency_stop(void)
{
    (void)hcics_axis_stop();
}

rt_bool_t hcics_axis_is_ready(void)
{
    return g_axis.ready;
}

rt_err_t hcics_axis_init(void)
{
    rt_err_t result;

    if (g_axis.initialized)
    {
        return g_axis.ready ? RT_EOK : -RT_ENOSYS;
    }

    rt_memset(&g_axis, 0, sizeof(g_axis));
    result = rt_mutex_init(&g_axis.lock, "haxis", RT_IPC_FLAG_FIFO);
    if (result != RT_EOK)
    {
        LOG_E("axis mutex init failed result=%d", result);
        return result;
    }

    g_axis.tx_pin = HCICS_AXIS_SOFT_TX_PIN;
    rt_pin_mode(g_axis.tx_pin, PIN_MODE_OUTPUT);
    rt_pin_write(g_axis.tx_pin, PIN_HIGH);
    rt_thread_mdelay(2);
    g_axis.ready = RT_TRUE;

    result = hcics_axis_enable();
    if (result != RT_EOK)
    {
        LOG_E("axis enable failed result=%d", result);
        goto init_failed;
    }

    result = hcics_axis_stop();
    if (result != RT_EOK)
    {
        LOG_E("axis stop failed result=%d", result);
        goto init_failed;
    }

    g_axis.initialized = RT_TRUE;
    LOG_I("axis ready soft_tx=%s baud=%u id=%u",
          HCICS_AXIS_SOFT_TX_PIN_NAME,
          (unsigned int)HCICS_AXIS_SOFT_BAUD,
          HCICS_AXIS_MOTOR_ID);
    return RT_EOK;

init_failed:
    g_axis.ready = RT_FALSE;
    g_axis.initialized = RT_TRUE;
    return result;
}
