/*
 * HCICS cleaner guard input.
 *
 * Production guard source: VL53L0X on RT-Thread I2C plus an optional contact
 * switch.  This module feeds hcics_cleaner_update_guard() with fresh data.
 */

#include "hcics_guard.h"

#include "hcics_cleaner.h"

#include <rtdevice.h>
#include "drv_common.h"

#define DBG_TAG "hcics.guard"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#ifndef HCICS_GUARD_I2C_BUS_NAME
#define HCICS_GUARD_I2C_BUS_NAME           "i2c1"
#endif

#ifndef HCICS_GUARD_VL53L0X_ADDR
#define HCICS_GUARD_VL53L0X_ADDR           0x29U
#endif

#ifndef HCICS_GUARD_THREAD_STACK
#define HCICS_GUARD_THREAD_STACK           2048
#endif

#ifndef HCICS_GUARD_THREAD_PRIO
#define HCICS_GUARD_THREAD_PRIO            20
#endif

#ifndef HCICS_GUARD_SAMPLE_MS
#define HCICS_GUARD_SAMPLE_MS              80U
#endif

#ifndef HCICS_GUARD_I2C_TIMEOUT_MS
#define HCICS_GUARD_I2C_TIMEOUT_MS         80U
#endif

#ifndef HCICS_GUARD_MAX_READ_ERRORS
#define HCICS_GUARD_MAX_READ_ERRORS        3U
#endif

#ifndef HCICS_GUARD_CONTACT_PIN
#define HCICS_GUARD_CONTACT_PIN            (-1)
#endif

#ifndef HCICS_GUARD_CONTACT_ACTIVE_LEVEL
#define HCICS_GUARD_CONTACT_ACTIVE_LEVEL   PIN_LOW
#endif

#define VL53_REG_SYSRANGE_START                    0x00U
#define VL53_REG_SYSTEM_SEQUENCE_CONFIG            0x01U
#define VL53_REG_SYSTEM_INTERRUPT_CONFIG_GPIO      0x0AU
#define VL53_REG_SYSTEM_INTERRUPT_CLEAR            0x0BU
#define VL53_REG_RESULT_INTERRUPT_STATUS           0x13U
#define VL53_REG_RESULT_RANGE_STATUS               0x14U
#define VL53_REG_MSRC_CONFIG_CONTROL               0x60U
#define VL53_REG_FINAL_RANGE_MIN_RATE_RTN_LIMIT    0x44U
#define VL53_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET  0x4FU
#define VL53_REG_DYNAMIC_SPAD_NUM_REQUESTED        0x4EU
#define VL53_REG_GLOBAL_CONFIG_REF_EN_START_SELECT 0xB6U
#define VL53_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0  0xB0U
#define VL53_REG_GPIO_HV_MUX_ACTIVE_HIGH           0x84U
#define VL53_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV  0x89U
#define VL53_REG_IDENTIFICATION_MODEL_ID           0xC0U

typedef struct
{
    rt_bool_t initialized;
    rt_bool_t ready;
    rt_uint8_t stop_variable;
    rt_uint8_t read_errors;
    struct rt_i2c_bus_device *bus;
    rt_thread_t thread;
} hcics_guard_context_t;

static hcics_guard_context_t g_guard;

#ifdef RT_USING_I2C

static rt_tick_t hcics_guard_timeout_ticks(void)
{
    return rt_tick_from_millisecond(HCICS_GUARD_I2C_TIMEOUT_MS);
}

static rt_err_t hcics_guard_write_multi(rt_uint8_t reg,
                                        const rt_uint8_t *data,
                                        rt_uint16_t len)
{
    rt_uint8_t buffer[8];
    struct rt_i2c_msg msg;

    if ((g_guard.bus == RT_NULL) || (data == RT_NULL) || (len == 0U))
    {
        return -RT_EINVAL;
    }
    if (len > (sizeof(buffer) - 1U))
    {
        return -RT_EINVAL;
    }

    buffer[0] = reg;
    rt_memcpy(&buffer[1], data, len);

    msg.addr = HCICS_GUARD_VL53L0X_ADDR;
    msg.flags = RT_I2C_WR;
    msg.len = (rt_uint16_t)(len + 1U);
    msg.buf = buffer;

    return (rt_i2c_transfer(g_guard.bus, &msg, 1) == 1) ? RT_EOK : -RT_ERROR;
}

static rt_err_t hcics_guard_read_multi(rt_uint8_t reg,
                                       rt_uint8_t *data,
                                       rt_uint16_t len)
{
    struct rt_i2c_msg msgs[2];

    if ((g_guard.bus == RT_NULL) || (data == RT_NULL) || (len == 0U))
    {
        return -RT_EINVAL;
    }

    msgs[0].addr = HCICS_GUARD_VL53L0X_ADDR;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].len = 1;
    msgs[0].buf = &reg;
    msgs[1].addr = HCICS_GUARD_VL53L0X_ADDR;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].len = len;
    msgs[1].buf = data;

    return (rt_i2c_transfer(g_guard.bus, msgs, 2) == 2) ? RT_EOK : -RT_ERROR;
}

static rt_err_t hcics_guard_write_u8(rt_uint8_t reg, rt_uint8_t value)
{
    return hcics_guard_write_multi(reg, &value, 1);
}

static rt_err_t hcics_guard_read_u8(rt_uint8_t reg, rt_uint8_t *value)
{
    return hcics_guard_read_multi(reg, value, 1);
}

static rt_err_t hcics_guard_write_u16(rt_uint8_t reg, rt_uint16_t value)
{
    rt_uint8_t data[2];

    data[0] = (rt_uint8_t)(value >> 8);
    data[1] = (rt_uint8_t)(value & 0xFFU);
    return hcics_guard_write_multi(reg, data, 2);
}

static rt_err_t hcics_guard_read_u16(rt_uint8_t reg, rt_uint16_t *value)
{
    rt_uint8_t data[2];
    rt_err_t result;

    if (value == RT_NULL)
    {
        return -RT_EINVAL;
    }

    result = hcics_guard_read_multi(reg, data, 2);
    if (result != RT_EOK)
    {
        return result;
    }

    *value = (rt_uint16_t)(((rt_uint16_t)data[0] << 8) | data[1]);
    return RT_EOK;
}

static rt_err_t hcics_guard_update_u8(rt_uint8_t reg,
                                      rt_uint8_t clear_mask,
                                      rt_uint8_t set_mask)
{
    rt_uint8_t value;
    rt_err_t result;

    result = hcics_guard_read_u8(reg, &value);
    if (result != RT_EOK)
    {
        return result;
    }

    value = (rt_uint8_t)((value & (rt_uint8_t)(~clear_mask)) | set_mask);
    return hcics_guard_write_u8(reg, value);
}

static rt_bool_t hcics_guard_wait_reg_bits(rt_uint8_t reg,
                                           rt_uint8_t mask,
                                           rt_bool_t set)
{
    rt_tick_t start = rt_tick_get();
    rt_tick_t timeout = hcics_guard_timeout_ticks();

    while ((rt_tick_get() - start) <= timeout)
    {
        rt_uint8_t value = 0U;

        if (hcics_guard_read_u8(reg, &value) != RT_EOK)
        {
            return RT_FALSE;
        }
        if (set)
        {
            if ((value & mask) != 0U)
            {
                return RT_TRUE;
            }
        }
        else if ((value & mask) == 0U)
        {
            return RT_TRUE;
        }

        rt_thread_mdelay(2);
    }

    return RT_FALSE;
}

static rt_err_t hcics_guard_write_tuning(void)
{
    static const rt_uint8_t seq[][2] =
    {
        {0xFF, 0x01}, {0x00, 0x00}, {0xFF, 0x00}, {0x09, 0x00},
        {0x10, 0x00}, {0x11, 0x00}, {0x24, 0x01}, {0x25, 0xFF},
        {0x75, 0x00}, {0xFF, 0x01}, {0x4E, 0x2C}, {0x48, 0x00},
        {0x30, 0x20}, {0xFF, 0x00}, {0x30, 0x09}, {0x54, 0x00},
        {0x31, 0x04}, {0x32, 0x03}, {0x40, 0x83}, {0x46, 0x25},
        {0x60, 0x00}, {0x27, 0x00}, {0x50, 0x06}, {0x51, 0x00},
        {0x52, 0x96}, {0x56, 0x08}, {0x57, 0x30}, {0x61, 0x00},
        {0x62, 0x00}, {0x64, 0x00}, {0x65, 0x00}, {0x66, 0xA0},
        {0xFF, 0x01}, {0x22, 0x32}, {0x47, 0x14}, {0x49, 0xFF},
        {0x4A, 0x00}, {0xFF, 0x00}, {0x7A, 0x0A}, {0x7B, 0x00},
        {0x78, 0x21}, {0xFF, 0x01}, {0x23, 0x34}, {0x42, 0x00},
        {0x44, 0xFF}, {0x45, 0x26}, {0x46, 0x05}, {0x40, 0x40},
        {0x0E, 0x06}, {0x20, 0x1A}, {0x43, 0x40}, {0xFF, 0x00},
        {0x34, 0x03}, {0x35, 0x44}, {0xFF, 0x01}, {0x31, 0x04},
        {0x4B, 0x09}, {0x4C, 0x05}, {0x4D, 0x04}, {0xFF, 0x00},
        {0x44, 0x00}, {0x45, 0x20}, {0x47, 0x08}, {0x48, 0x28},
        {0x67, 0x00}, {0x70, 0x04}, {0x71, 0x01}, {0x72, 0xFE},
        {0x76, 0x00}, {0x77, 0x00}, {0xFF, 0x01}, {0x0D, 0x01},
        {0xFF, 0x00}, {0x80, 0x01}, {0x01, 0xF8}, {0xFF, 0x01},
        {0x8E, 0x01}, {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00},
    };
    rt_size_t i;

    for (i = 0; i < (sizeof(seq) / sizeof(seq[0])); i++)
    {
        rt_err_t result = hcics_guard_write_u8(seq[i][0], seq[i][1]);
        if (result != RT_EOK)
        {
            return result;
        }
    }

    return RT_EOK;
}

static rt_err_t hcics_guard_get_spad_info(rt_uint8_t *count,
                                          rt_bool_t *type_is_aperture)
{
    rt_uint8_t tmp;
    rt_tick_t start;
    rt_tick_t timeout;

    if ((count == RT_NULL) || (type_is_aperture == RT_NULL))
    {
        return -RT_EINVAL;
    }

    if ((hcics_guard_write_u8(0x80, 0x01) != RT_EOK) ||
        (hcics_guard_write_u8(0xFF, 0x01) != RT_EOK) ||
        (hcics_guard_write_u8(0x00, 0x00) != RT_EOK) ||
        (hcics_guard_write_u8(0xFF, 0x06) != RT_EOK) ||
        (hcics_guard_update_u8(0x83, 0x00, 0x04) != RT_EOK) ||
        (hcics_guard_write_u8(0xFF, 0x07) != RT_EOK) ||
        (hcics_guard_write_u8(0x81, 0x01) != RT_EOK) ||
        (hcics_guard_write_u8(0x80, 0x01) != RT_EOK) ||
        (hcics_guard_write_u8(0x94, 0x6B) != RT_EOK) ||
        (hcics_guard_write_u8(0x83, 0x00) != RT_EOK))
    {
        return -RT_ERROR;
    }

    start = rt_tick_get();
    timeout = hcics_guard_timeout_ticks();
    do
    {
        if (hcics_guard_read_u8(0x83, &tmp) != RT_EOK)
        {
            return -RT_ERROR;
        }
        if (tmp != 0U)
        {
            break;
        }
        rt_thread_mdelay(2);
    } while ((rt_tick_get() - start) <= timeout);

    if (tmp == 0U)
    {
        return -RT_ETIMEOUT;
    }

    if ((hcics_guard_write_u8(0x83, 0x01) != RT_EOK) ||
        (hcics_guard_read_u8(0x92, &tmp) != RT_EOK))
    {
        return -RT_ERROR;
    }

    *count = (rt_uint8_t)(tmp & 0x7FU);
    *type_is_aperture = ((tmp & 0x80U) != 0U) ? RT_TRUE : RT_FALSE;

    if ((hcics_guard_write_u8(0x81, 0x00) != RT_EOK) ||
        (hcics_guard_write_u8(0xFF, 0x06) != RT_EOK) ||
        (hcics_guard_update_u8(0x83, 0x04, 0x00) != RT_EOK) ||
        (hcics_guard_write_u8(0xFF, 0x01) != RT_EOK) ||
        (hcics_guard_write_u8(0x00, 0x01) != RT_EOK) ||
        (hcics_guard_write_u8(0xFF, 0x00) != RT_EOK) ||
        (hcics_guard_write_u8(0x80, 0x00) != RT_EOK))
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t hcics_guard_config_spads(void)
{
    rt_uint8_t spad_count = 0U;
    rt_bool_t aperture = RT_FALSE;
    rt_uint8_t ref_spad_map[6];
    rt_uint8_t first_spad;
    rt_uint8_t enabled = 0U;
    rt_uint8_t i;
    rt_err_t result;

    result = hcics_guard_get_spad_info(&spad_count, &aperture);
    if (result != RT_EOK)
    {
        return result;
    }

    result = hcics_guard_read_multi(VL53_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0,
                                    ref_spad_map, sizeof(ref_spad_map));
    if (result != RT_EOK)
    {
        return result;
    }

    if ((hcics_guard_write_u8(0xFF, 0x01) != RT_EOK) ||
        (hcics_guard_write_u8(VL53_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00) != RT_EOK) ||
        (hcics_guard_write_u8(VL53_REG_DYNAMIC_SPAD_NUM_REQUESTED, 0x2C) != RT_EOK) ||
        (hcics_guard_write_u8(0xFF, 0x00) != RT_EOK) ||
        (hcics_guard_write_u8(VL53_REG_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4) != RT_EOK))
    {
        return -RT_ERROR;
    }

    first_spad = aperture ? 12U : 0U;
    for (i = 0U; i < 48U; i++)
    {
        rt_uint8_t byte = (rt_uint8_t)(i / 8U);
        rt_uint8_t bit = (rt_uint8_t)(1U << (i % 8U));

        if ((i < first_spad) || (enabled == spad_count))
        {
            ref_spad_map[byte] &= (rt_uint8_t)(~bit);
        }
        else if ((ref_spad_map[byte] & bit) != 0U)
        {
            enabled++;
        }
    }

    return hcics_guard_write_multi(VL53_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0,
                                   ref_spad_map, sizeof(ref_spad_map));
}

static rt_err_t hcics_guard_ref_calibration(rt_uint8_t vhv_init_byte)
{
    if (hcics_guard_write_u8(VL53_REG_SYSRANGE_START,
                             (rt_uint8_t)(0x01U | vhv_init_byte)) != RT_EOK)
    {
        return -RT_ERROR;
    }

    if (!hcics_guard_wait_reg_bits(VL53_REG_RESULT_INTERRUPT_STATUS, 0x07U, RT_TRUE))
    {
        return -RT_ETIMEOUT;
    }

    if ((hcics_guard_write_u8(VL53_REG_SYSTEM_INTERRUPT_CLEAR, 0x01) != RT_EOK) ||
        (hcics_guard_write_u8(VL53_REG_SYSRANGE_START, 0x00) != RT_EOK))
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t hcics_guard_sensor_init(void)
{
    rt_uint8_t id = 0U;
    rt_err_t result;

    result = hcics_guard_read_u8(VL53_REG_IDENTIFICATION_MODEL_ID, &id);
    if (result != RT_EOK)
    {
        LOG_E("VL53L0X probe failed on %s", HCICS_GUARD_I2C_BUS_NAME);
        return result;
    }
    if (id != 0xEEU)
    {
        LOG_E("unexpected VL53L0X model id=0x%02x", id);
        return -RT_ENOSYS;
    }

    if ((hcics_guard_update_u8(VL53_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV,
                               0x00, 0x01) != RT_EOK) ||
        (hcics_guard_write_u8(0x88, 0x00) != RT_EOK) ||
        (hcics_guard_write_u8(0x80, 0x01) != RT_EOK) ||
        (hcics_guard_write_u8(0xFF, 0x01) != RT_EOK) ||
        (hcics_guard_write_u8(0x00, 0x00) != RT_EOK) ||
        (hcics_guard_read_u8(0x91, &g_guard.stop_variable) != RT_EOK) ||
        (hcics_guard_write_u8(0x00, 0x01) != RT_EOK) ||
        (hcics_guard_write_u8(0xFF, 0x00) != RT_EOK) ||
        (hcics_guard_write_u8(0x80, 0x00) != RT_EOK) ||
        (hcics_guard_update_u8(VL53_REG_MSRC_CONFIG_CONTROL, 0x00, 0x12) != RT_EOK) ||
        (hcics_guard_write_u16(VL53_REG_FINAL_RANGE_MIN_RATE_RTN_LIMIT, 32U) != RT_EOK) ||
        (hcics_guard_write_u8(VL53_REG_SYSTEM_SEQUENCE_CONFIG, 0xFF) != RT_EOK))
    {
        return -RT_ERROR;
    }

    result = hcics_guard_config_spads();
    if (result != RT_EOK)
    {
        return result;
    }

    if ((hcics_guard_write_tuning() != RT_EOK) ||
        (hcics_guard_write_u8(VL53_REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04) != RT_EOK) ||
        (hcics_guard_update_u8(VL53_REG_GPIO_HV_MUX_ACTIVE_HIGH, 0x10, 0x00) != RT_EOK) ||
        (hcics_guard_write_u8(VL53_REG_SYSTEM_INTERRUPT_CLEAR, 0x01) != RT_EOK) ||
        (hcics_guard_write_u8(VL53_REG_SYSTEM_SEQUENCE_CONFIG, 0x01) != RT_EOK))
    {
        return -RT_ERROR;
    }

    result = hcics_guard_ref_calibration(0x40);
    if (result != RT_EOK)
    {
        return result;
    }

    if (hcics_guard_write_u8(VL53_REG_SYSTEM_SEQUENCE_CONFIG, 0x02) != RT_EOK)
    {
        return -RT_ERROR;
    }

    result = hcics_guard_ref_calibration(0x00);
    if (result != RT_EOK)
    {
        return result;
    }

    return hcics_guard_write_u8(VL53_REG_SYSTEM_SEQUENCE_CONFIG, 0xE8);
}

static rt_err_t hcics_guard_read_distance(rt_uint16_t *distance_mm)
{
    if (distance_mm == RT_NULL)
    {
        return -RT_EINVAL;
    }

    if ((hcics_guard_write_u8(0x80, 0x01) != RT_EOK) ||
        (hcics_guard_write_u8(0xFF, 0x01) != RT_EOK) ||
        (hcics_guard_write_u8(0x00, 0x00) != RT_EOK) ||
        (hcics_guard_write_u8(0x91, g_guard.stop_variable) != RT_EOK) ||
        (hcics_guard_write_u8(0x00, 0x01) != RT_EOK) ||
        (hcics_guard_write_u8(0xFF, 0x00) != RT_EOK) ||
        (hcics_guard_write_u8(0x80, 0x00) != RT_EOK) ||
        (hcics_guard_write_u8(VL53_REG_SYSRANGE_START, 0x01) != RT_EOK))
    {
        return -RT_ERROR;
    }

    if (!hcics_guard_wait_reg_bits(VL53_REG_SYSRANGE_START, 0x01U, RT_FALSE))
    {
        return -RT_ETIMEOUT;
    }
    if (!hcics_guard_wait_reg_bits(VL53_REG_RESULT_INTERRUPT_STATUS, 0x07U, RT_TRUE))
    {
        return -RT_ETIMEOUT;
    }
    if (hcics_guard_read_u16((rt_uint8_t)(VL53_REG_RESULT_RANGE_STATUS + 10U),
                             distance_mm) != RT_EOK)
    {
        return -RT_ERROR;
    }

    return hcics_guard_write_u8(VL53_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
}

#else

static rt_err_t hcics_guard_sensor_init(void)
{
    LOG_E("RT_USING_I2C is disabled; VL53L0X guard unavailable");
    return -RT_ENOSYS;
}

static rt_err_t hcics_guard_read_distance(rt_uint16_t *distance_mm)
{
    RT_UNUSED(distance_mm);
    return -RT_ENOSYS;
}

#endif

static rt_bool_t hcics_guard_read_contact(void)
{
#if HCICS_GUARD_CONTACT_PIN >= 0
    return (rt_pin_read(HCICS_GUARD_CONTACT_PIN) == HCICS_GUARD_CONTACT_ACTIVE_LEVEL) ?
           RT_TRUE : RT_FALSE;
#else
    return RT_FALSE;
#endif
}

static rt_err_t hcics_guard_publish(rt_uint16_t distance_mm,
                                    rt_bool_t tof_valid,
                                    rt_bool_t force_stop)
{
    hcics_cleaner_guard_t guard;

    rt_memset(&guard, 0, sizeof(guard));
    guard.tof_mm = tof_valid ? distance_mm : HCICS_CLEANER_TOF_INVALID_MM;
    guard.tof_valid = tof_valid;
    guard.contact = hcics_guard_read_contact();
    guard.force_stop = force_stop;
    guard.tick = rt_tick_get();
    return hcics_cleaner_update_guard(&guard);
}

static rt_err_t hcics_guard_sample_once(void)
{
    rt_uint16_t distance = HCICS_CLEANER_TOF_INVALID_MM;
    rt_err_t result;

    result = hcics_guard_read_distance(&distance);
    if ((result == RT_EOK) && (distance != 0U) &&
        (distance != HCICS_CLEANER_TOF_INVALID_MM))
    {
        g_guard.read_errors = 0U;
        return hcics_guard_publish(distance, RT_TRUE, RT_FALSE);
    }

    if (g_guard.read_errors < 0xFFU)
    {
        g_guard.read_errors++;
    }

    if (g_guard.read_errors >= HCICS_GUARD_MAX_READ_ERRORS)
    {
        LOG_E("guard distance read failed result=%d errors=%u",
              result, (unsigned int)g_guard.read_errors);
        (void)hcics_guard_publish(HCICS_CLEANER_TOF_INVALID_MM,
                                  RT_FALSE,
                                  RT_TRUE);
    }

    return result;
}

static void hcics_guard_thread(void *parameter)
{
    RT_UNUSED(parameter);

    while (1)
    {
        (void)hcics_guard_sample_once();
        rt_thread_mdelay(HCICS_GUARD_SAMPLE_MS);
    }
}

rt_err_t hcics_guard_init(void)
{
    rt_err_t result;

    if (g_guard.initialized)
    {
        return RT_EOK;
    }

    rt_memset(&g_guard, 0, sizeof(g_guard));

#ifdef RT_USING_I2C
    g_guard.bus = rt_i2c_bus_device_find(HCICS_GUARD_I2C_BUS_NAME);
    if (g_guard.bus == RT_NULL)
    {
        LOG_E("I2C bus %s not found", HCICS_GUARD_I2C_BUS_NAME);
        return -RT_ENOSYS;
    }
#endif

#if HCICS_GUARD_CONTACT_PIN >= 0
    rt_pin_mode(HCICS_GUARD_CONTACT_PIN, PIN_MODE_INPUT_PULLUP);
#endif

    result = hcics_guard_sensor_init();
    if (result != RT_EOK)
    {
        LOG_E("VL53L0X guard init failed result=%d", result);
        return result;
    }

    result = hcics_guard_sample_once();
    if (result != RT_EOK)
    {
        LOG_E("VL53L0X first sample failed result=%d", result);
        return result;
    }

    g_guard.thread = rt_thread_create("hguard",
                                      hcics_guard_thread,
                                      RT_NULL,
                                      HCICS_GUARD_THREAD_STACK,
                                      HCICS_GUARD_THREAD_PRIO,
                                      20);
    if (g_guard.thread == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    rt_thread_startup(g_guard.thread);
    g_guard.ready = RT_TRUE;
    g_guard.initialized = RT_TRUE;
    LOG_I("guard ready: VL53L0X bus=%s scl=PH15 sda=PH8 addr=0x%02x contact_pin=%d",
          HCICS_GUARD_I2C_BUS_NAME,
          HCICS_GUARD_VL53L0X_ADDR,
          HCICS_GUARD_CONTACT_PIN);
    return RT_EOK;
}

rt_bool_t hcics_guard_is_ready(void)
{
    return g_guard.ready;
}
