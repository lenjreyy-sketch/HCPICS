/*
 * HCICS path tracking module.
 *
 * The module consumes the same camera target stream as the application.
 * Only frames with target.mode == HCICS_CAM_MODE_TRACK are used for tracking.
 */

#ifndef HCICS_TRACK_H
#define HCICS_TRACK_H

#include <rtthread.h>
#include "hcics_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HCICS_TRACK_TARGET_HOME 0U
#define HCICS_TRACK_TARGET_MIN  1U
#define HCICS_TRACK_TARGET_MAX  9U

typedef enum
{
    HCICS_TRACK_ACTION_FORWARD = 0,
    HCICS_TRACK_ACTION_TURN_LEFT,
    HCICS_TRACK_ACTION_TURN_RIGHT,
    HCICS_TRACK_ACTION_UTURN,
    HCICS_TRACK_ACTION_FINISH_DELAY,
    HCICS_TRACK_ACTION_STOP_AT_TARGET
} hcics_track_action_t;

typedef enum
{
    HCICS_TRACK_STATE_LINE = 0,
    HCICS_TRACK_STATE_LEFT_TURN = 1,
    HCICS_TRACK_STATE_RIGHT_TURN = 2,
    HCICS_TRACK_STATE_UTURN = 3,
    HCICS_TRACK_STATE_PAUSED = 88,
    HCICS_TRACK_STATE_FINISHED = 99,
    HCICS_TRACK_STATE_IDLE = 255
} hcics_track_state_t;

typedef void (*hcics_track_set_motor_t)(rt_int16_t left, rt_int16_t right, void *user);
typedef void (*hcics_track_get_speed_t)(rt_int16_t *left, rt_int16_t *right, void *user);

typedef struct
{
    hcics_track_set_motor_t set_motor;
    hcics_track_get_speed_t get_speed;
    void *user;
} hcics_track_motion_ops_t;

typedef struct
{
    rt_bool_t initialized;
    rt_bool_t running;
    rt_bool_t finished;
    rt_bool_t target_locked;
    rt_bool_t boost_active;
    rt_uint8_t target_id;
    rt_uint8_t current_node_index;
    rt_uint8_t path_len;
    rt_uint8_t visual_mission_state;
    rt_uint8_t current_action;
    rt_uint32_t s_current_cooldown_ms;
    rt_uint32_t last_tick_ms;
    rt_int16_t left_cmd;
    rt_int16_t right_cmd;
    rt_int16_t actual_left;
    rt_int16_t actual_right;
    float error;
    float filtered_error;
    float d_error;
    float turn;
    float base_speed;
    const char *path_name;
    const char *step_name;
} hcics_track_status_t;

extern float Kp;
extern float Ki;
extern float Kd;
extern volatile rt_uint8_t current_node_index;
extern volatile rt_uint8_t visual_mission_state;
extern volatile rt_uint32_t s_current_cooldown_ms;

rt_err_t hcics_track_init(const hcics_track_motion_ops_t *ops);
rt_err_t hcics_track_start_path(rt_uint8_t target_id);
void hcics_track_stop(void);
void hcics_track_update_camera(const hcics_camera_target_t *target);
void hcics_track_tick(rt_uint32_t now_ms);
rt_bool_t hcics_track_is_finished(void);
void hcics_track_status(hcics_track_status_t *status);
const char *hcics_track_state_name(rt_uint8_t state);

void hcics_track_motion_set(rt_int16_t left, rt_int16_t right);
void hcics_track_motion_get_realtime(rt_int16_t *left, rt_int16_t *right);

#ifdef __cplusplus
}
#endif

#endif /* HCICS_TRACK_H */
