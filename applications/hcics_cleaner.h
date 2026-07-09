/*
 * HCICS gimbal cleaner module.
 */

#ifndef HCICS_CLEANER_H
#define HCICS_CLEANER_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HCICS_CLEANER_CAMERA_MODE_TRACK   1U
#define HCICS_CLEANER_CAMERA_MODE_CLEAN   2U
#define HCICS_CLEANER_TOF_INVALID_MM      0xFFFFU

typedef enum
{
    HCICS_CLEANER_STATE_IDLE = 0,
    HCICS_CLEANER_STATE_AIMING,
    HCICS_CLEANER_STATE_EXTENDING,
    HCICS_CLEANER_STATE_SPRAYING,
    HCICS_CLEANER_STATE_SCRUBBING,
    HCICS_CLEANER_STATE_VERIFYING,
    HCICS_CLEANER_STATE_RETRACTING,
    HCICS_CLEANER_STATE_DONE,
    HCICS_CLEANER_STATE_STOPPED,
    HCICS_CLEANER_STATE_ERROR
} hcics_cleaner_state_t;

typedef struct
{
    rt_uint8_t mode;
    rt_uint16_t x;
    rt_uint16_t y;
    rt_uint8_t width;
    rt_uint8_t type;
    rt_bool_t valid;
    rt_tick_t tick;
} hcics_cleaner_camera_target_t;

typedef struct
{
    rt_uint16_t tof_mm;
    rt_bool_t tof_valid;
    rt_bool_t contact;
    rt_bool_t force_stop;
    rt_tick_t tick;
} hcics_cleaner_guard_t;

typedef struct
{
    const char *pwm_device_name;
    int y_pwm_channel;
    int z_pwm_channel;
    rt_uint32_t pwm_period_ns;
    rt_uint32_t y_safe_ns;
    rt_uint32_t y_center_ns;
    rt_uint32_t y_min_ns;
    rt_uint32_t y_max_ns;
    rt_uint32_t z_retract_ns;
    rt_uint32_t z_extend_ns;
    rt_uint32_t z_wash_ns;
    rt_base_t relay_pin;
    rt_base_t relay_on_level;
    rt_base_t relay_off_level;
} hcics_cleaner_config_t;

typedef struct
{
    hcics_cleaner_state_t state;
    rt_bool_t initialized;
    rt_bool_t busy;
    rt_bool_t stop_requested;
    rt_bool_t relay_on;
    rt_err_t last_result;
    rt_uint8_t cycle;
    rt_uint8_t max_cycles;
    rt_uint32_t y_pulse_ns;
    rt_uint32_t z_pulse_ns;
    hcics_cleaner_camera_target_t camera;
    hcics_cleaner_guard_t guard;
} hcics_cleaner_status_t;

void hcics_cleaner_get_default_config(hcics_cleaner_config_t *config);
rt_err_t hcics_cleaner_init(const hcics_cleaner_config_t *config);
rt_err_t hcics_cleaner_update_camera_target(const hcics_cleaner_camera_target_t *target);
rt_err_t hcics_cleaner_update_guard(const hcics_cleaner_guard_t *guard);
rt_err_t hcics_cleaner_start(rt_uint8_t max_cycles);
void hcics_cleaner_emergency_stop(void);
void hcics_cleaner_reset(void);
rt_err_t hcics_cleaner_get_status(hcics_cleaner_status_t *status);
const char *hcics_cleaner_state_name(hcics_cleaner_state_t state);

#ifdef __cplusplus
}
#endif

#endif /* HCICS_CLEANER_H */
