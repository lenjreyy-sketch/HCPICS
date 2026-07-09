/*
 * HCICS chassis motion adapter.
 *
 * The adapter keeps legacy xunjian wheel-command semantics in a small
 * RT-Thread-facing module: signed left/right targets, ramp limiting,
 * emergency stop, actual-speed feedback hooks, and a required hardware output
 * path for the chassis.
 */

#ifndef HCICS_MOTION_H
#define HCICS_MOTION_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HCICS_MOTION_SPEED_MAX          250
#define HCICS_MOTION_DEFAULT_RAMP_STEP  15

typedef enum
{
    HCICS_MOTION_ACTUAL_NONE = 0,
    HCICS_MOTION_ACTUAL_EXTERNAL,
    HCICS_MOTION_ACTUAL_ENCODER
} hcics_motion_actual_source_t;

typedef rt_err_t (*hcics_motion_output_cb_t)(rt_int16_t left,
                                             rt_int16_t right,
                                             void *user_data);

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
    rt_uint32_t boost_active_ms;
    rt_tick_t last_update_tick;
} hcics_motion_snapshot_t;

rt_err_t hcics_motion_init(void);
rt_err_t hcics_motion_set_output_callback(hcics_motion_output_cb_t callback,
                                          void *user_data);

rt_err_t hcics_motion_set_target(rt_int16_t left, rt_int16_t right);
rt_err_t hcics_motion_set_target_direct(rt_int16_t left, rt_int16_t right);
rt_err_t hcics_motion_update(void);

void hcics_motion_emergency_stop(rt_uint8_t reason);
void hcics_motion_clear_emergency_stop(void);
void hcics_motion_reset(void);

rt_err_t hcics_motion_set_actual_speed(rt_int16_t left, rt_int16_t right);
rt_err_t hcics_motion_set_encoder_delta(rt_int16_t left_delta,
                                        rt_int16_t right_delta);
void hcics_motion_get_realtime(rt_int16_t *target_left,
                               rt_int16_t *actual_left,
                               rt_int16_t *target_right,
                               rt_int16_t *actual_right);
void hcics_motion_get_snapshot(hcics_motion_snapshot_t *snapshot);
void hcics_motion_status_dump(void);

#ifdef __cplusplus
}
#endif

#endif /* HCICS_MOTION_H */
