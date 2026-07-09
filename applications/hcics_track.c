/*
 * HCICS path tracking module.
 *
 * This is the RT-Thread port of the legacy xunjian path scheduler.  The
 * hardware contract is intentionally single-camera: only mode 1 frames are
 * consumed here, while mode 2 frames belong to the cleaner module.
 */

#include "hcics_track.h"

#include <stdlib.h>

#define DBG_TAG "hcics.track"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#ifndef HCICS_TRACK_CAMERA_CENTER_X
#define HCICS_TRACK_CAMERA_CENTER_X        80
#endif

#ifndef HCICS_TRACK_BASE_SPEED
#define HCICS_TRACK_BASE_SPEED             54.0f
#endif

#ifndef HCICS_TRACK_MIN_TARGET_W
#define HCICS_TRACK_MIN_TARGET_W           3U
#endif

#ifndef HCICS_TRACK_LOW_CONF_W
#define HCICS_TRACK_LOW_CONF_W             32U
#endif

#ifndef HCICS_TRACK_NODE_WIDE_W
#define HCICS_TRACK_NODE_WIDE_W            80U
#endif

#ifndef HCICS_TRACK_NODE_WIDE_FALLBACK_W
#define HCICS_TRACK_NODE_WIDE_FALLBACK_W   100U
#endif

#ifndef HCICS_TRACK_NODE_COOLDOWN_MS
#define HCICS_TRACK_NODE_COOLDOWN_MS       500U
#endif

#ifndef HCICS_TRACK_TURN_LEFT_MS
#define HCICS_TRACK_TURN_LEFT_MS           900U
#endif

#ifndef HCICS_TRACK_TURN_RIGHT_MS
#define HCICS_TRACK_TURN_RIGHT_MS          750U
#endif

#ifndef HCICS_TRACK_UTURN_MS
#define HCICS_TRACK_UTURN_MS               1200U
#endif

#ifndef HCICS_TRACK_FINISH_DELAY_MS
#define HCICS_TRACK_FINISH_DELAY_MS        400U
#endif

#ifndef HCICS_TRACK_TARGET_STALE_MS
#define HCICS_TRACK_TARGET_STALE_MS        1500U
#endif

#ifndef HCICS_TRACK_TICK_WARN_MS
#define HCICS_TRACK_TICK_WARN_MS           100U
#endif

#ifndef HCICS_TRACK_TURN_LIMIT
#define HCICS_TRACK_TURN_LIMIT             70.0f
#endif

#ifndef HCICS_TRACK_LOW_CONF_TURN_LIMIT
#define HCICS_TRACK_LOW_CONF_TURN_LIMIT    10.0f
#endif

#ifndef HCICS_TRACK_SEARCH_SPEED
#define HCICS_TRACK_SEARCH_SPEED           27
#endif

#ifndef HCICS_TRACK_LEFT_INNER_SPEED
#define HCICS_TRACK_LEFT_INNER_SPEED       (-25)
#endif

#ifndef HCICS_TRACK_LEFT_OUTER_SPEED
#define HCICS_TRACK_LEFT_OUTER_SPEED       85
#endif

#ifndef HCICS_TRACK_RIGHT_INNER_SPEED
#define HCICS_TRACK_RIGHT_INNER_SPEED      (-25)
#endif

#ifndef HCICS_TRACK_RIGHT_OUTER_SPEED
#define HCICS_TRACK_RIGHT_OUTER_SPEED      85
#endif

#ifndef HCICS_TRACK_UTURN_LEFT_SPEED
#define HCICS_TRACK_UTURN_LEFT_SPEED       (-150)
#endif

#ifndef HCICS_TRACK_UTURN_RIGHT_SPEED
#define HCICS_TRACK_UTURN_RIGHT_SPEED      150
#endif

#ifndef HCICS_TRACK_FILTER_ALPHA
#define HCICS_TRACK_FILTER_ALPHA           0.6f
#endif

#define HCICS_TRACK_GRID_COLS              3U
#define HCICS_TRACK_MAX_PATH_STEPS         16U

typedef enum
{
    HCICS_TRACK_PATH_HOME = 0,
    HCICS_TRACK_PATH_TO_TARGET,
    HCICS_TRACK_PATH_RETURN_HOME
} hcics_track_path_id_t;

typedef struct
{
    hcics_track_action_t action;
    const char *name;
} hcics_track_step_t;

typedef struct
{
    const hcics_track_step_t *steps;
    rt_uint8_t len;
    hcics_track_path_id_t id;
    rt_uint8_t destination;
    const char *name;
} hcics_track_path_t;

typedef struct
{
    rt_bool_t initialized;
    rt_bool_t running;
    rt_bool_t finished;
    rt_bool_t target_locked;
    rt_bool_t node_latched;
    rt_bool_t action_active;

    rt_uint8_t target_id;
    rt_uint8_t current_location;
    rt_uint8_t path_index;
    rt_uint8_t active_action;

    rt_uint32_t last_tick_ms;
    rt_uint32_t last_node_ms;
    rt_uint32_t action_end_ms;

    rt_int16_t left_cmd;
    rt_int16_t right_cmd;
    rt_int16_t actual_left;
    rt_int16_t actual_right;

    float error;
    float last_error;
    float filtered_error;
    float d_error;
    float turn;
    float base_speed;

    hcics_camera_target_t camera;
    hcics_track_motion_ops_t ops;
    const hcics_track_path_t *path;
    hcics_track_path_t planned_path;
    hcics_track_step_t planned_steps[HCICS_TRACK_MAX_PATH_STEPS];
    struct rt_mutex lock;
} hcics_track_context_t;

float Kp = 0.35f;
float Ki = 0.0f;
float Kd = 35.0f;
volatile rt_uint8_t current_node_index = 0U;
volatile rt_uint8_t visual_mission_state = HCICS_TRACK_STATE_IDLE;
volatile rt_uint32_t s_current_cooldown_ms = HCICS_TRACK_NODE_COOLDOWN_MS;

static hcics_track_context_t g_track;

static rt_uint32_t hcics_track_now_ms(void)
{
    return (rt_uint32_t)(((rt_uint64_t)rt_tick_get() * 1000ULL) / RT_TICK_PER_SECOND);
}

static rt_int16_t hcics_track_float_to_i16(float value)
{
    if (value > 32767.0f)
    {
        return 32767;
    }
    if (value < -32768.0f)
    {
        return -32768;
    }

    return (rt_int16_t)value;
}

static float hcics_track_clampf(float value, float min_value, float max_value)
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

static rt_bool_t hcics_track_target_to_grid(rt_uint8_t target_id,
                                            rt_uint8_t *row,
                                            rt_uint8_t *col)
{
    if ((target_id < HCICS_TRACK_TARGET_MIN) ||
        (target_id > HCICS_TRACK_TARGET_MAX))
    {
        return RT_FALSE;
    }

    if (row != RT_NULL)
    {
        *row = (rt_uint8_t)(((target_id - 1U) / HCICS_TRACK_GRID_COLS) + 1U);
    }
    if (col != RT_NULL)
    {
        *col = (rt_uint8_t)(((target_id - 1U) % HCICS_TRACK_GRID_COLS) + 1U);
    }

    return RT_TRUE;
}

static rt_bool_t hcics_track_motor_ready(void)
{
    return (g_track.initialized && (g_track.ops.set_motor != RT_NULL)) ?
           RT_TRUE : RT_FALSE;
}

static rt_err_t hcics_track_add_step(hcics_track_action_t action,
                                     const char *name)
{
    if (g_track.planned_path.len >= HCICS_TRACK_MAX_PATH_STEPS)
    {
        return -RT_EFULL;
    }

    g_track.planned_steps[g_track.planned_path.len].action = action;
    g_track.planned_steps[g_track.planned_path.len].name = name;
    g_track.planned_path.len++;
    return RT_EOK;
}

static rt_err_t hcics_track_build_to_target(rt_uint8_t target_id)
{
    rt_uint8_t row;
    rt_uint8_t col;
    rt_uint8_t i;

    if (!hcics_track_target_to_grid(target_id, &row, &col))
    {
        return -RT_EINVAL;
    }

    g_track.planned_path.id = HCICS_TRACK_PATH_TO_TARGET;
    g_track.planned_path.destination = target_id;
    g_track.planned_path.name = "auto-to-target";

    for (i = 0U; i < row; i++)
    {
        if (hcics_track_add_step(HCICS_TRACK_ACTION_FORWARD,
                                 "main lane forward") != RT_EOK)
        {
            return -RT_EFULL;
        }
    }

    if (hcics_track_add_step(HCICS_TRACK_ACTION_TURN_RIGHT,
                             "enter target row") != RT_EOK)
    {
        return -RT_EFULL;
    }

    for (i = 1U; i < col; i++)
    {
        if (hcics_track_add_step(HCICS_TRACK_ACTION_FORWARD,
                                 "row lane forward") != RT_EOK)
        {
            return -RT_EFULL;
        }
    }

    return hcics_track_add_step(HCICS_TRACK_ACTION_STOP_AT_TARGET,
                                "stop at fault panel");
}

static rt_err_t hcics_track_build_return_home(void)
{
    rt_uint8_t row;
    rt_uint8_t col;
    rt_uint8_t i;

    if (g_track.current_location == HCICS_TRACK_TARGET_HOME)
    {
        g_track.planned_path.id = HCICS_TRACK_PATH_HOME;
        g_track.planned_path.destination = HCICS_TRACK_TARGET_HOME;
        g_track.planned_path.name = "already-home";
        return hcics_track_add_step(HCICS_TRACK_ACTION_FINISH_DELAY,
                                    "home settle");
    }

    if (!hcics_track_target_to_grid(g_track.current_location, &row, &col))
    {
        return -RT_EINVAL;
    }

    g_track.planned_path.id = HCICS_TRACK_PATH_RETURN_HOME;
    g_track.planned_path.destination = HCICS_TRACK_TARGET_HOME;
    g_track.planned_path.name = "auto-return-home";

    if (hcics_track_add_step(HCICS_TRACK_ACTION_UTURN,
                             "turn around after cleaning") != RT_EOK)
    {
        return -RT_EFULL;
    }

    for (i = 1U; i < col; i++)
    {
        if (hcics_track_add_step(HCICS_TRACK_ACTION_FORWARD,
                                 "row lane return") != RT_EOK)
        {
            return -RT_EFULL;
        }
    }

    if (hcics_track_add_step(HCICS_TRACK_ACTION_TURN_LEFT,
                             "exit row to main lane") != RT_EOK)
    {
        return -RT_EFULL;
    }

    for (i = 0U; i < row; i++)
    {
        if (hcics_track_add_step(HCICS_TRACK_ACTION_FORWARD,
                                 "main lane return") != RT_EOK)
        {
            return -RT_EFULL;
        }
    }

    return hcics_track_add_step(HCICS_TRACK_ACTION_FINISH_DELAY,
                                "dock home");
}

static rt_err_t hcics_track_plan_path(rt_uint8_t target_id)
{
    rt_memset(g_track.planned_steps, 0, sizeof(g_track.planned_steps));
    rt_memset(&g_track.planned_path, 0, sizeof(g_track.planned_path));
    g_track.planned_path.steps = g_track.planned_steps;

    if (target_id == HCICS_TRACK_TARGET_HOME)
    {
        return hcics_track_build_return_home();
    }

    return hcics_track_build_to_target(target_id);
}

static const char *hcics_track_action_name(rt_uint8_t action)
{
    switch (action)
    {
    case HCICS_TRACK_ACTION_FORWARD: return "FORWARD";
    case HCICS_TRACK_ACTION_TURN_LEFT: return "TURN_LEFT";
    case HCICS_TRACK_ACTION_TURN_RIGHT: return "TURN_RIGHT";
    case HCICS_TRACK_ACTION_UTURN: return "UTURN";
    case HCICS_TRACK_ACTION_FINISH_DELAY: return "FINISH_DELAY";
    case HCICS_TRACK_ACTION_STOP_AT_TARGET: return "STOP_AT_TARGET";
    default: return "UNKNOWN";
    }
}

const char *hcics_track_state_name(rt_uint8_t state)
{
    switch (state)
    {
    case HCICS_TRACK_STATE_LINE: return "LINE";
    case HCICS_TRACK_STATE_LEFT_TURN: return "LEFT_TURN";
    case HCICS_TRACK_STATE_RIGHT_TURN: return "RIGHT_TURN";
    case HCICS_TRACK_STATE_UTURN: return "UTURN";
    case HCICS_TRACK_STATE_PAUSED: return "PAUSED";
    case HCICS_TRACK_STATE_FINISHED: return "FINISHED";
    case HCICS_TRACK_STATE_IDLE: return "IDLE";
    default: return "UNKNOWN";
    }
}

void hcics_track_motion_set(rt_int16_t left, rt_int16_t right)
{
    g_track.left_cmd = left;
    g_track.right_cmd = right;

    if (g_track.ops.set_motor == RT_NULL)
    {
        LOG_E("track motor output is not configured");
        return;
    }

    g_track.ops.set_motor(left, right, g_track.ops.user);
}

void hcics_track_motion_get_realtime(rt_int16_t *left, rt_int16_t *right)
{
    if (g_track.ops.get_speed != RT_NULL)
    {
        g_track.ops.get_speed(left, right, g_track.ops.user);
        return;
    }

    if (left != RT_NULL)
    {
        *left = g_track.actual_left;
    }
    if (right != RT_NULL)
    {
        *right = g_track.actual_right;
    }
}

static rt_bool_t hcics_track_target_recent(const hcics_camera_target_t *target,
                                           rt_uint32_t now_ms)
{
    rt_uint32_t target_ms;

    if ((target == RT_NULL) || (target->mode != HCICS_CAM_MODE_TRACK) ||
        (target->tick == 0U) || (target->width < HCICS_TRACK_MIN_TARGET_W))
    {
        return RT_FALSE;
    }

    target_ms = (rt_uint32_t)(((rt_uint64_t)target->tick * 1000ULL) /
                              RT_TICK_PER_SECOND);
    return ((now_ms - target_ms) <= HCICS_TRACK_TARGET_STALE_MS) ? RT_TRUE : RT_FALSE;
}

static rt_bool_t hcics_track_node_seen(const hcics_camera_target_t *target,
                                       rt_uint32_t now_ms)
{
    rt_bool_t typed_node;
    rt_bool_t fallback_node;

    if (!hcics_track_target_recent(target, now_ms))
    {
        return RT_FALSE;
    }

    if ((now_ms - g_track.last_node_ms) < s_current_cooldown_ms)
    {
        return RT_FALSE;
    }

    typed_node = ((target->type == 3U) || (target->type == 4U) ||
                  (target->type == 7U)) ? RT_TRUE : RT_FALSE;
    fallback_node = ((target->type == 0U) &&
                     (target->width >= HCICS_TRACK_NODE_WIDE_FALLBACK_W)) ?
                    RT_TRUE : RT_FALSE;

    return ((typed_node && (target->width >= HCICS_TRACK_NODE_WIDE_W)) ||
            fallback_node) ? RT_TRUE : RT_FALSE;
}

static void hcics_track_finish_path(rt_uint32_t now_ms)
{
    rt_uint8_t destination = HCICS_TRACK_TARGET_HOME;

    if (g_track.path != RT_NULL)
    {
        destination = g_track.path->destination;
    }

    g_track.running = RT_FALSE;
    g_track.finished = RT_TRUE;
    g_track.action_active = RT_FALSE;
    g_track.current_location = destination;
    g_track.last_node_ms = now_ms;
    visual_mission_state = HCICS_TRACK_STATE_FINISHED;
    hcics_track_motion_set(0, 0);

    LOG_I("path finished target=%u location=%u", g_track.target_id,
          g_track.current_location);
}

static void hcics_track_begin_action(const hcics_track_step_t *step,
                                     rt_uint32_t now_ms)
{
    rt_uint32_t duration_ms = 0U;

    if (step == RT_NULL)
    {
        return;
    }

    g_track.active_action = (rt_uint8_t)step->action;
    g_track.action_active = RT_FALSE;

    switch (step->action)
    {
    case HCICS_TRACK_ACTION_FORWARD:
        g_track.last_node_ms = now_ms;
        visual_mission_state = HCICS_TRACK_STATE_LINE;
        LOG_I("node=%u action=%s step=%s", current_node_index,
              hcics_track_action_name(step->action), step->name);
        break;

    case HCICS_TRACK_ACTION_TURN_LEFT:
        duration_ms = HCICS_TRACK_TURN_LEFT_MS;
        g_track.action_active = RT_TRUE;
        g_track.action_end_ms = now_ms + duration_ms;
        visual_mission_state = HCICS_TRACK_STATE_LEFT_TURN;
        hcics_track_motion_set(HCICS_TRACK_LEFT_INNER_SPEED,
                               HCICS_TRACK_LEFT_OUTER_SPEED);
        LOG_I("node=%u left turn %u ms step=%s", current_node_index,
              duration_ms, step->name);
        break;

    case HCICS_TRACK_ACTION_TURN_RIGHT:
        duration_ms = HCICS_TRACK_TURN_RIGHT_MS;
        g_track.action_active = RT_TRUE;
        g_track.action_end_ms = now_ms + duration_ms;
        visual_mission_state = HCICS_TRACK_STATE_RIGHT_TURN;
        hcics_track_motion_set(HCICS_TRACK_RIGHT_OUTER_SPEED,
                               HCICS_TRACK_RIGHT_INNER_SPEED);
        LOG_I("node=%u right turn %u ms step=%s", current_node_index,
              duration_ms, step->name);
        break;

    case HCICS_TRACK_ACTION_UTURN:
        g_track.action_active = RT_TRUE;
        g_track.action_end_ms = now_ms + HCICS_TRACK_UTURN_MS;
        visual_mission_state = HCICS_TRACK_STATE_UTURN;
        hcics_track_motion_set(HCICS_TRACK_UTURN_LEFT_SPEED,
                               HCICS_TRACK_UTURN_RIGHT_SPEED);
        LOG_I("node=%u uturn %u ms step=%s", current_node_index,
              HCICS_TRACK_UTURN_MS, step->name);
        break;

    case HCICS_TRACK_ACTION_FINISH_DELAY:
        g_track.action_active = RT_TRUE;
        g_track.action_end_ms = now_ms + HCICS_TRACK_FINISH_DELAY_MS;
        visual_mission_state = HCICS_TRACK_STATE_PAUSED;
        hcics_track_motion_set(0, 0);
        LOG_I("node=%u final delay step=%s", current_node_index, step->name);
        break;

    case HCICS_TRACK_ACTION_STOP_AT_TARGET:
        hcics_track_finish_path(now_ms);
        LOG_I("node=%u stop at target step=%s", current_node_index, step->name);
        break;

    default:
        LOG_W("unknown action=%u", step->action);
        break;
    }
}

static void hcics_track_complete_action(rt_uint32_t now_ms)
{
    if (!g_track.action_active)
    {
        return;
    }

    if ((rt_int32_t)(now_ms - g_track.action_end_ms) < 0)
    {
        return;
    }

    if (g_track.active_action == HCICS_TRACK_ACTION_FINISH_DELAY)
    {
        hcics_track_finish_path(now_ms);
        return;
    }

    g_track.action_active = RT_FALSE;
    g_track.last_node_ms = now_ms;
    visual_mission_state = HCICS_TRACK_STATE_LINE;
    hcics_track_motion_set((rt_int16_t)HCICS_TRACK_BASE_SPEED,
                           (rt_int16_t)HCICS_TRACK_BASE_SPEED);
    LOG_I("action done, resume line");
}

static void hcics_track_accept_node(rt_uint32_t now_ms)
{
    const hcics_track_step_t *step;

    if ((g_track.path == RT_NULL) || (g_track.path_index >= g_track.path->len))
    {
        hcics_track_finish_path(now_ms);
        return;
    }

    step = &g_track.path->steps[g_track.path_index];
    current_node_index = g_track.path_index;
    g_track.path_index++;
    hcics_track_begin_action(step, now_ms);
}

static void hcics_track_apply_line_follow(rt_uint32_t now_ms)
{
    float base = HCICS_TRACK_BASE_SPEED;
    float turn;
    float left;
    float right;
    rt_bool_t recent = hcics_track_target_recent(&g_track.camera, now_ms);

    g_track.base_speed = base;
    g_track.target_locked = recent;

    if (!recent)
    {
        g_track.turn = 0.0f;
        g_track.left_cmd = 0;
        g_track.right_cmd = 0;
        g_track.base_speed = 0.0f;
        hcics_track_motion_set(0, 0);
        return;
    }

    if (recent)
    {
        g_track.error = ((float)((int)g_track.camera.x - HCICS_TRACK_CAMERA_CENTER_X));
        g_track.filtered_error =
            (HCICS_TRACK_FILTER_ALPHA * g_track.error) +
            ((1.0f - HCICS_TRACK_FILTER_ALPHA) * g_track.filtered_error);
        g_track.d_error = g_track.filtered_error - g_track.last_error;
        g_track.last_error = g_track.filtered_error;

        turn = (Kp * g_track.filtered_error) + (Kd * g_track.d_error);
        if (g_track.camera.width < HCICS_TRACK_LOW_CONF_W)
        {
            turn = hcics_track_clampf(turn,
                                      -HCICS_TRACK_LOW_CONF_TURN_LIMIT,
                                      HCICS_TRACK_LOW_CONF_TURN_LIMIT);
            base = (float)HCICS_TRACK_SEARCH_SPEED;
        }
        else
        {
            turn = hcics_track_clampf(turn,
                                      -HCICS_TRACK_TURN_LIMIT,
                                      HCICS_TRACK_TURN_LIMIT);
        }
    }
    g_track.turn = turn;
    left = base - turn;
    right = base + turn;
    g_track.left_cmd = hcics_track_float_to_i16(left);
    g_track.right_cmd = hcics_track_float_to_i16(right);

    hcics_track_motion_set(g_track.left_cmd, g_track.right_cmd);
}

rt_err_t hcics_track_init(const hcics_track_motion_ops_t *ops)
{
    rt_err_t result;

    if (g_track.initialized)
    {
        if (ops != RT_NULL)
        {
            if (ops->set_motor == RT_NULL)
            {
                return -RT_EINVAL;
            }
            g_track.ops = *ops;
        }
        return RT_EOK;
    }

    if ((ops == RT_NULL) || (ops->set_motor == RT_NULL))
    {
        LOG_E("track init requires motor output ops");
        return -RT_EINVAL;
    }

    rt_memset(&g_track, 0, sizeof(g_track));
    result = rt_mutex_init(&g_track.lock, "htrk", RT_IPC_FLAG_FIFO);
    if (result != RT_EOK)
    {
        LOG_E("track mutex init failed result=%d", result);
        return result;
    }
    g_track.ops = *ops;
    g_track.initialized = RT_TRUE;
    g_track.current_location = HCICS_TRACK_TARGET_HOME;
    g_track.base_speed = HCICS_TRACK_BASE_SPEED;
    visual_mission_state = HCICS_TRACK_STATE_IDLE;

    LOG_I("track module initialized");
    return RT_EOK;
}

rt_err_t hcics_track_start_path(rt_uint8_t target_id)
{
    rt_err_t result;
    rt_uint32_t now_ms;

    if (!hcics_track_motor_ready())
    {
        LOG_E("track path start denied: motor output not ready");
        return -RT_ENOSYS;
    }

    now_ms = hcics_track_now_ms();
    rt_mutex_take(&g_track.lock, RT_WAITING_FOREVER);
    if (!hcics_track_target_recent(&g_track.camera, now_ms))
    {
        rt_mutex_release(&g_track.lock);
        LOG_E("track path start denied: mode=1 camera not fresh");
        return -RT_ETIMEOUT;
    }

    result = hcics_track_plan_path(target_id);
    if (result != RT_EOK)
    {
        rt_mutex_release(&g_track.lock);
        return result;
    }

    g_track.path = &g_track.planned_path;
    g_track.target_id = target_id;
    g_track.path_index = 0U;
    g_track.running = RT_TRUE;
    g_track.finished = RT_FALSE;
    g_track.action_active = RT_FALSE;
    g_track.node_latched = RT_FALSE;
    g_track.last_tick_ms = now_ms;
    g_track.last_node_ms = now_ms - s_current_cooldown_ms;
    g_track.filtered_error = 0.0f;
    g_track.last_error = 0.0f;
    g_track.turn = 0.0f;
    current_node_index = 0U;
    visual_mission_state = HCICS_TRACK_STATE_LINE;
    rt_mutex_release(&g_track.lock);

    LOG_I("start path=%s target=%u len=%u",
          g_track.planned_path.name, target_id, g_track.planned_path.len);
    return RT_EOK;
}

void hcics_track_stop(void)
{
    if (!g_track.initialized)
    {
        return;
    }

    rt_mutex_take(&g_track.lock, RT_WAITING_FOREVER);
    g_track.running = RT_FALSE;
    g_track.finished = RT_FALSE;
    g_track.action_active = RT_FALSE;
    visual_mission_state = HCICS_TRACK_STATE_IDLE;
    hcics_track_motion_set(0, 0);
    rt_mutex_release(&g_track.lock);
}

void hcics_track_update_camera(const hcics_camera_target_t *target)
{
    if ((target == RT_NULL) || (target->mode != HCICS_CAM_MODE_TRACK))
    {
        return;
    }
    if (!g_track.initialized)
    {
        return;
    }

    rt_mutex_take(&g_track.lock, RT_WAITING_FOREVER);
    g_track.camera = *target;
    g_track.target_locked = (target->width >= HCICS_TRACK_MIN_TARGET_W) ?
                            RT_TRUE : RT_FALSE;
    rt_mutex_release(&g_track.lock);
}

void hcics_track_tick(rt_uint32_t now_ms)
{
    if (!g_track.initialized)
    {
        return;
    }

    rt_mutex_take(&g_track.lock, RT_WAITING_FOREVER);
    if (!g_track.running)
    {
        rt_mutex_release(&g_track.lock);
        return;
    }

    if ((now_ms - g_track.last_tick_ms) > HCICS_TRACK_TICK_WARN_MS)
    {
        LOG_D("slow tick gap=%u", now_ms - g_track.last_tick_ms);
    }
    g_track.last_tick_ms = now_ms;

    if (g_track.action_active)
    {
        hcics_track_complete_action(now_ms);
        rt_mutex_release(&g_track.lock);
        return;
    }

    if (hcics_track_node_seen(&g_track.camera, now_ms))
    {
        hcics_track_accept_node(now_ms);
        rt_mutex_release(&g_track.lock);
        return;
    }

    hcics_track_apply_line_follow(now_ms);
    rt_mutex_release(&g_track.lock);
}

rt_bool_t hcics_track_is_finished(void)
{
    rt_bool_t finished;

    if (!g_track.initialized)
    {
        return RT_FALSE;
    }

    rt_mutex_take(&g_track.lock, RT_WAITING_FOREVER);
    finished = g_track.finished;
    rt_mutex_release(&g_track.lock);
    return finished;
}

void hcics_track_status(hcics_track_status_t *status)
{
    const hcics_track_step_t *step = RT_NULL;

    if (status == RT_NULL)
    {
        return;
    }
    if (!g_track.initialized)
    {
        rt_memset(status, 0, sizeof(*status));
        status->visual_mission_state = HCICS_TRACK_STATE_IDLE;
        status->path_name = "not-initialized";
        status->step_name = "none";
        return;
    }

    rt_mutex_take(&g_track.lock, RT_WAITING_FOREVER);
    rt_memset(status, 0, sizeof(*status));
    status->initialized = g_track.initialized;
    status->running = g_track.running;
    status->finished = g_track.finished;
    status->target_locked = g_track.target_locked;
    status->target_id = g_track.target_id;
    status->current_node_index = current_node_index;
    status->path_len = (g_track.path != RT_NULL) ? g_track.path->len : 0U;
    status->visual_mission_state = visual_mission_state;
    status->current_action = g_track.active_action;
    status->s_current_cooldown_ms = s_current_cooldown_ms;
    status->last_tick_ms = g_track.last_tick_ms;
    status->left_cmd = g_track.left_cmd;
    status->right_cmd = g_track.right_cmd;
    status->actual_left = g_track.actual_left;
    status->actual_right = g_track.actual_right;
    status->error = g_track.error;
    status->filtered_error = g_track.filtered_error;
    status->d_error = g_track.d_error;
    status->turn = g_track.turn;
    status->base_speed = g_track.base_speed;
    status->path_name = (g_track.path != RT_NULL) ? g_track.path->name : "none";
    if ((g_track.path != RT_NULL) && (g_track.path_index < g_track.path->len))
    {
        step = &g_track.path->steps[g_track.path_index];
    }
    status->step_name = (step != RT_NULL) ? step->name : "none";
    rt_mutex_release(&g_track.lock);
}
