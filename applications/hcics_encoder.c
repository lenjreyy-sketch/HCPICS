/*
 * HCICS chassis quadrature encoder adapter.
 *
 * ART-Pi does not provide a STM32H7 timer-encoder BSP device in this tree, so
 * the two wheel encoders are registered as RT-Thread pulse_encoder devices by
 * decoding AB phase transitions on GPIO interrupts.
 */

#include "hcics_encoder.h"

#include <rtdevice.h>
#include "drv_common.h"

#define DBG_TAG "hcics.encoder"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#if defined(RT_USING_PULSE_ENCODER) && defined(RT_USING_PIN)
#include <drivers/pulse_encoder.h>

#ifndef HCICS_ENCODER_LEFT_A_PIN
#define HCICS_ENCODER_LEFT_A_PIN       GET_PIN(H, 2)
#endif

#ifndef HCICS_ENCODER_LEFT_B_PIN
#define HCICS_ENCODER_LEFT_B_PIN       GET_PIN(H, 3)
#endif

#ifndef HCICS_ENCODER_RIGHT_A_PIN
#define HCICS_ENCODER_RIGHT_A_PIN      GET_PIN(H, 13)
#endif

#ifndef HCICS_ENCODER_RIGHT_B_PIN
#define HCICS_ENCODER_RIGHT_B_PIN      GET_PIN(H, 14)
#endif

#ifndef HCICS_ENCODER_LEFT_REVERSED
#define HCICS_ENCODER_LEFT_REVERSED    0
#endif

#ifndef HCICS_ENCODER_RIGHT_REVERSED
#define HCICS_ENCODER_RIGHT_REVERSED   0
#endif

#ifndef HCICS_ENCODER_INPUT_MODE
#define HCICS_ENCODER_INPUT_MODE       PIN_MODE_INPUT_PULLUP
#endif

typedef struct
{
    struct rt_pulse_encoder_device pulse;
    const char *name;
    rt_base_t pin_a;
    rt_base_t pin_b;
    rt_bool_t reversed;
    rt_bool_t attached;
    rt_bool_t enabled;
    volatile rt_int32_t count;
    rt_uint8_t last_state;
} hcics_encoder_device_t;

static rt_err_t hcics_encoder_device_init(struct rt_pulse_encoder_device *pulse);
static rt_int32_t hcics_encoder_get_count(struct rt_pulse_encoder_device *pulse);
static rt_err_t hcics_encoder_clear_count(struct rt_pulse_encoder_device *pulse);
static rt_err_t hcics_encoder_control(struct rt_pulse_encoder_device *pulse,
                                      rt_uint32_t cmd,
                                      void *args);

static const struct rt_pulse_encoder_ops g_hcics_encoder_ops =
{
    hcics_encoder_device_init,
    hcics_encoder_get_count,
    hcics_encoder_clear_count,
    hcics_encoder_control
};

static hcics_encoder_device_t g_left_encoder =
{
    .name = "pulse1",
    .pin_a = HCICS_ENCODER_LEFT_A_PIN,
    .pin_b = HCICS_ENCODER_LEFT_B_PIN,
    .reversed = HCICS_ENCODER_LEFT_REVERSED ? RT_TRUE : RT_FALSE,
};

static hcics_encoder_device_t g_right_encoder =
{
    .name = "pulse2",
    .pin_a = HCICS_ENCODER_RIGHT_A_PIN,
    .pin_b = HCICS_ENCODER_RIGHT_B_PIN,
    .reversed = HCICS_ENCODER_RIGHT_REVERSED ? RT_TRUE : RT_FALSE,
};

static rt_bool_t g_encoder_registered;

static rt_uint8_t hcics_encoder_read_state(hcics_encoder_device_t *encoder)
{
    rt_uint8_t a = (rt_pin_read(encoder->pin_a) == PIN_HIGH) ? 1U : 0U;
    rt_uint8_t b = (rt_pin_read(encoder->pin_b) == PIN_HIGH) ? 1U : 0U;

    return (rt_uint8_t)((a << 1) | b);
}

static void hcics_encoder_irq(void *args)
{
    static const rt_int8_t decode_table[16] =
    {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0
    };
    hcics_encoder_device_t *encoder = (hcics_encoder_device_t *)args;
    rt_uint8_t state;
    rt_uint8_t index;
    rt_int8_t delta;

    if ((encoder == RT_NULL) || !encoder->enabled)
    {
        return;
    }

    state = hcics_encoder_read_state(encoder);
    index = (rt_uint8_t)((encoder->last_state << 2) | state);
    encoder->last_state = state;

    delta = decode_table[index & 0x0FU];
    if (delta == 0)
    {
        return;
    }

    if (encoder->reversed)
    {
        delta = (rt_int8_t)(-delta);
    }

    encoder->count += delta;
}

static rt_err_t hcics_encoder_attach_irqs(hcics_encoder_device_t *encoder)
{
    rt_err_t result;

    if (encoder->attached)
    {
        return RT_EOK;
    }

    result = rt_pin_attach_irq(encoder->pin_a,
                               PIN_IRQ_MODE_RISING_FALLING,
                               hcics_encoder_irq,
                               encoder);
    if (result != RT_EOK)
    {
        LOG_E("%s attach A irq failed pin=%d result=%d",
              encoder->name, (int)encoder->pin_a, result);
        return result;
    }

    result = rt_pin_attach_irq(encoder->pin_b,
                               PIN_IRQ_MODE_RISING_FALLING,
                               hcics_encoder_irq,
                               encoder);
    if (result != RT_EOK)
    {
        (void)rt_pin_detach_irq(encoder->pin_a);
        LOG_E("%s attach B irq failed pin=%d result=%d",
              encoder->name, (int)encoder->pin_b, result);
        return result;
    }

    encoder->attached = RT_TRUE;
    return RT_EOK;
}

static rt_err_t hcics_encoder_enable_irqs(hcics_encoder_device_t *encoder)
{
    rt_err_t result;

    result = rt_pin_irq_enable(encoder->pin_a, PIN_IRQ_ENABLE);
    if (result != RT_EOK)
    {
        LOG_E("%s enable A irq failed pin=%d result=%d",
              encoder->name, (int)encoder->pin_a, result);
        return result;
    }

    result = rt_pin_irq_enable(encoder->pin_b, PIN_IRQ_ENABLE);
    if (result != RT_EOK)
    {
        (void)rt_pin_irq_enable(encoder->pin_a, PIN_IRQ_DISABLE);
        LOG_E("%s enable B irq failed pin=%d result=%d",
              encoder->name, (int)encoder->pin_b, result);
        return result;
    }

    return RT_EOK;
}

static rt_err_t hcics_encoder_device_init(struct rt_pulse_encoder_device *pulse)
{
    hcics_encoder_device_t *encoder = (hcics_encoder_device_t *)pulse;
    rt_base_t level;

    if (encoder == RT_NULL)
    {
        return -RT_EINVAL;
    }

    rt_pin_mode(encoder->pin_a, HCICS_ENCODER_INPUT_MODE);
    rt_pin_mode(encoder->pin_b, HCICS_ENCODER_INPUT_MODE);

    level = rt_hw_interrupt_disable();
    encoder->last_state = hcics_encoder_read_state(encoder);
    encoder->count = 0;
    rt_hw_interrupt_enable(level);

    return RT_EOK;
}

static rt_int32_t hcics_encoder_get_count(struct rt_pulse_encoder_device *pulse)
{
    hcics_encoder_device_t *encoder = (hcics_encoder_device_t *)pulse;
    rt_base_t level;
    rt_int32_t count;

    if (encoder == RT_NULL)
    {
        return 0;
    }

    level = rt_hw_interrupt_disable();
    count = encoder->count;
    rt_hw_interrupt_enable(level);

    return count;
}

static rt_err_t hcics_encoder_clear_count(struct rt_pulse_encoder_device *pulse)
{
    hcics_encoder_device_t *encoder = (hcics_encoder_device_t *)pulse;
    rt_base_t level;

    if (encoder == RT_NULL)
    {
        return -RT_EINVAL;
    }

    level = rt_hw_interrupt_disable();
    encoder->count = 0;
    encoder->last_state = hcics_encoder_read_state(encoder);
    rt_hw_interrupt_enable(level);

    return RT_EOK;
}

static rt_err_t hcics_encoder_control(struct rt_pulse_encoder_device *pulse,
                                      rt_uint32_t cmd,
                                      void *args)
{
    hcics_encoder_device_t *encoder = (hcics_encoder_device_t *)pulse;
    rt_err_t result;

    (void)args;

    if (encoder == RT_NULL)
    {
        return -RT_EINVAL;
    }

    switch (cmd)
    {
    case PULSE_ENCODER_CMD_ENABLE:
        result = hcics_encoder_device_init(pulse);
        if (result != RT_EOK)
        {
            return result;
        }
        result = hcics_encoder_attach_irqs(encoder);
        if (result != RT_EOK)
        {
            return result;
        }
        encoder->enabled = RT_TRUE;
        result = hcics_encoder_enable_irqs(encoder);
        if (result != RT_EOK)
        {
            encoder->enabled = RT_FALSE;
            return result;
        }
        LOG_I("%s enabled A=%d B=%d reversed=%u",
              encoder->name,
              (int)encoder->pin_a,
              (int)encoder->pin_b,
              (unsigned int)encoder->reversed);
        return RT_EOK;

    case PULSE_ENCODER_CMD_DISABLE:
        encoder->enabled = RT_FALSE;
        (void)rt_pin_irq_enable(encoder->pin_a, PIN_IRQ_DISABLE);
        (void)rt_pin_irq_enable(encoder->pin_b, PIN_IRQ_DISABLE);
        return RT_EOK;

    default:
        return -RT_ENOSYS;
    }
}

static rt_err_t hcics_encoder_register_one(hcics_encoder_device_t *encoder)
{
    rt_err_t result;

    if (rt_device_find(encoder->name) != RT_NULL)
    {
        LOG_E("%s already exists", encoder->name);
        return -RT_ERROR;
    }

    encoder->pulse.ops = &g_hcics_encoder_ops;
    encoder->pulse.type = AB_PHASE_PULSE_ENCODER;
    result = rt_device_pulse_encoder_register(&encoder->pulse,
                                              encoder->name,
                                              encoder);
    if (result != RT_EOK)
    {
        LOG_E("%s register failed result=%d", encoder->name, result);
        return result;
    }

    LOG_I("%s registered A=%d B=%d",
          encoder->name, (int)encoder->pin_a, (int)encoder->pin_b);
    return RT_EOK;
}

rt_err_t hcics_encoder_init(void)
{
    rt_err_t result;

    if (g_encoder_registered)
    {
        return RT_EOK;
    }

    result = hcics_encoder_register_one(&g_left_encoder);
    if (result != RT_EOK)
    {
        return result;
    }

    result = hcics_encoder_register_one(&g_right_encoder);
    if (result != RT_EOK)
    {
        return result;
    }

    g_encoder_registered = RT_TRUE;
    return RT_EOK;
}

static int hcics_encoder_auto_init(void)
{
    return (int)hcics_encoder_init();
}
INIT_DEVICE_EXPORT(hcics_encoder_auto_init);

#else

rt_err_t hcics_encoder_init(void)
{
    LOG_E("RT_USING_PULSE_ENCODER and RT_USING_PIN are required");
    return -RT_ENOSYS;
}

#endif
