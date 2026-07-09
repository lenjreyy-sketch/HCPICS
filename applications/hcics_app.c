/*
 * HCICS photovoltaic cleaning vehicle application.
 *
 * This file owns the automatic RT-Thread mission flow.  Required hardware
 * devices are checked during startup; the mission is not allowed to run on
 * missing chassis, camera, or cleaning outputs.
 */

#include "hcics_app.h"

#include "hcics_axis.h"
#include "hcics_cleaner.h"
#include "hcics_guard.h"
#include "hcics_motion.h"
#include "hcics_track.h"

#include <rtdevice.h>
#include "drv_common.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#ifndef RT_USING_SAL
#error "HCICS production firmware requires RT_USING_SAL for UDP task input."
#endif

#ifndef RT_USING_WIFI
#error "HCICS production firmware requires RT_USING_WIFI for AP6212/CYWL6208 STA."
#endif

#ifdef RT_USING_SAL
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <unistd.h>
#endif

#ifdef RT_USING_WIFI
#include <wlan_mgnt.h>
extern int rt_hw_wlan_wait_init_done(rt_uint32_t time_ms);
#endif

#define DBG_TAG "hcics"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#ifdef HCICS_USE_LOCAL_CONFIG
#include "hcics_config.h"
#endif

#define HCICS_NET_PORT                 6208
#define HCICS_TRACK_CAMERA_UART_NAME   "uart1"
#define HCICS_CLEAN_CAMERA_UART_NAME   "uart6"

#ifndef HCICS_WIFI_SSID
#define HCICS_WIFI_SSID                "HCICS-ESP32S3"
#endif

#ifndef HCICS_WIFI_PASSWORD
#define HCICS_WIFI_PASSWORD            "change-me"
#endif

#ifndef HCICS_UDP_TOKEN
#define HCICS_UDP_TOKEN                "change-me"
#endif

#ifndef HCICS_WIFI_INIT_TIMEOUT_MS
#define HCICS_WIFI_INIT_TIMEOUT_MS     15000U
#endif

#ifndef HCICS_WIFI_CONNECT_TIMEOUT_MS
#define HCICS_WIFI_CONNECT_TIMEOUT_MS  20000U
#endif

#ifndef HCICS_WIFI_RETRY_DELAY_MS
#define HCICS_WIFI_RETRY_DELAY_MS      2000U
#endif

#ifndef HCICS_WIFI_POLL_MS
#define HCICS_WIFI_POLL_MS             200U
#endif

#define HCICS_EVENT_DEPTH              12
#define HCICS_THREAD_STACK             4096
#define HCICS_IO_THREAD_STACK          3072
#define HCICS_CONTROL_THREAD_STACK     2048
#define HCICS_THREAD_PRIO              17
#define HCICS_CONTROL_THREAD_PRIO      12
#define HCICS_IO_THREAD_PRIO           19
#define HCICS_CONTROL_PERIOD_MS        5U

#define HCICS_CLEAN_MAX_CYCLES         3U
#define HCICS_TRACK_POLL_MS            20U
#define HCICS_TRACK_NAV_TIMEOUT_MS     120000U
#define HCICS_TRACK_WAIT_CAMERA_MS     5000U
#define HCICS_TRACK_LOST_CAMERA_MS     2500U
#define HCICS_TRACK_MIN_CAMERA_W       3U
#define HCICS_CLEAN_POLL_MS            100U
#define HCICS_CLEAN_TIMEOUT_MS         60000U

typedef enum
{
    HCICS_MSG_FAULT = 1
} hcics_msg_type_t;

typedef struct
{
    hcics_msg_type_t type;
    hcics_fault_t fault;
#ifdef RT_USING_SAL
    struct sockaddr_in peer;
    rt_bool_t peer_valid;
#endif
} hcics_msg_t;

typedef enum
{
    HCICS_STATE_IDLE = 0,
    HCICS_STATE_NAV_TO_FAULT,
    HCICS_STATE_PARKED,
    HCICS_STATE_CLEANING,
    HCICS_STATE_VERIFY_CLEAR,
    HCICS_STATE_RETURN_HOME,
    HCICS_STATE_ERROR
} hcics_state_t;

typedef struct
{
    rt_bool_t initialized;
    rt_bool_t busy;
    volatile rt_bool_t abort;
    volatile rt_bool_t motion_fault;
    rt_uint8_t motion_fault_reason;
    hcics_state_t state;
    hcics_fault_t active_fault;
    hcics_camera_target_t latest_clean;
    hcics_camera_target_t latest_track;
    struct rt_mutex camera_lock;
    struct rt_mutex state_lock;
#ifdef RT_USING_SAL
    struct rt_mutex udp_lock;
    int udp_sock;
    rt_bool_t pending_report_valid;
    hcics_remote_cmd_t pending_report_cmd;
    rt_uint8_t pending_report_param;
    struct sockaddr_in pending_report_peer;
#endif
    rt_mq_t mq;
    rt_device_t track_camera_uart;
    rt_device_t clean_camera_uart;
} hcics_context_t;

static hcics_context_t g_hcics;

#ifdef RT_USING_SAL
static rt_bool_t hcics_wifi_link_ready(void);
static rt_bool_t hcics_pending_report_active(void);
#endif

static const char *hcics_state_name(hcics_state_t state)
{
    switch (state)
    {
    case HCICS_STATE_IDLE: return "IDLE";
    case HCICS_STATE_NAV_TO_FAULT: return "NAV_TO_FAULT";
    case HCICS_STATE_PARKED: return "PARKED";
    case HCICS_STATE_CLEANING: return "CLEANING";
    case HCICS_STATE_VERIFY_CLEAR: return "VERIFY_CLEAR";
    case HCICS_STATE_RETURN_HOME: return "RETURN_HOME";
    case HCICS_STATE_ERROR: return "ERROR";
    default: return "UNKNOWN";
    }
}

static void hcics_set_state(hcics_state_t state)
{
    g_hcics.state = state;
    LOG_I("state=%s", hcics_state_name(state));
}

static rt_err_t hcics_reserve_mission(const hcics_fault_t *fault)
{
    if ((fault == RT_NULL) || !hcics_protocol_fault_is_valid(fault))
    {
        return -RT_EINVAL;
    }

    rt_mutex_take(&g_hcics.state_lock, RT_WAITING_FOREVER);
    if (g_hcics.busy)
    {
        rt_mutex_release(&g_hcics.state_lock);
        return -RT_EBUSY;
    }
#ifdef RT_USING_SAL
    rt_mutex_take(&g_hcics.udp_lock, RT_WAITING_FOREVER);
    if (g_hcics.pending_report_valid)
    {
        rt_mutex_release(&g_hcics.udp_lock);
        rt_mutex_release(&g_hcics.state_lock);
        LOG_W("pending report blocks reserve id=%u", fault->id);
        return -RT_EBUSY;
    }
    rt_mutex_release(&g_hcics.udp_lock);
#endif

    g_hcics.busy = RT_TRUE;
    g_hcics.active_fault = *fault;
    rt_mutex_release(&g_hcics.state_lock);

    return RT_EOK;
}

static void hcics_release_mission(void)
{
    rt_mutex_take(&g_hcics.state_lock, RT_WAITING_FOREVER);
    g_hcics.busy = RT_FALSE;
    rt_mutex_release(&g_hcics.state_lock);
}

static rt_bool_t hcics_delay_abortable(rt_uint32_t ms)
{
    rt_uint32_t left = ms;

    while (left > 0U)
    {
        if (g_hcics.abort)
        {
            return RT_FALSE;
        }

        if (left >= 50U)
        {
            rt_thread_mdelay(50);
            left -= 50U;
        }
        else
        {
            rt_thread_mdelay(left);
            left = 0U;
        }
    }

    return RT_TRUE;
}

static rt_uint32_t hcics_now_ms(void)
{
    return (rt_uint32_t)(((rt_uint64_t)rt_tick_get() * 1000ULL) / RT_TICK_PER_SECOND);
}

static rt_uint32_t hcics_tick_to_ms(rt_tick_t tick)
{
    return (rt_uint32_t)(((rt_uint64_t)tick * 1000ULL) / RT_TICK_PER_SECOND);
}

static rt_bool_t hcics_latest_track_recent(rt_uint32_t max_age_ms)
{
    hcics_camera_target_t target;
    rt_uint32_t now_ms;
    rt_uint32_t target_ms;

    rt_mutex_take(&g_hcics.camera_lock, RT_WAITING_FOREVER);
    target = g_hcics.latest_track;
    rt_mutex_release(&g_hcics.camera_lock);

    if ((target.mode != HCICS_CAM_MODE_TRACK) ||
        (target.tick == 0U) ||
        (target.width < HCICS_TRACK_MIN_CAMERA_W))
    {
        return RT_FALSE;
    }

    now_ms = hcics_now_ms();
    target_ms = hcics_tick_to_ms(target.tick);
    return ((now_ms - target_ms) <= max_age_ms) ? RT_TRUE : RT_FALSE;
}

static rt_err_t hcics_wait_track_camera(rt_uint32_t timeout_ms)
{
    rt_uint32_t start_ms = hcics_now_ms();

    while (!g_hcics.abort)
    {
        if (hcics_latest_track_recent(HCICS_TRACK_LOST_CAMERA_MS))
        {
            return RT_EOK;
        }

        if ((hcics_now_ms() - start_ms) >= timeout_ms)
        {
            LOG_E("mode=1 track camera frame not ready");
            return -RT_ETIMEOUT;
        }

        rt_thread_mdelay(HCICS_TRACK_POLL_MS);
    }

    return -RT_EINTR;
}

static rt_bool_t hcics_track_state_requires_camera(rt_uint8_t state)
{
    return (state == HCICS_TRACK_STATE_LINE) ? RT_TRUE : RT_FALSE;
}

static void hcics_latch_motion_fault(rt_uint8_t reason)
{
    if (!g_hcics.motion_fault)
    {
        g_hcics.motion_fault_reason = reason;
        LOG_E("motion fault latched reason=%u", reason);
    }

    g_hcics.motion_fault = RT_TRUE;
    g_hcics.abort = RT_TRUE;
}

static void hcics_track_set_motor(rt_int16_t left, rt_int16_t right, void *user)
{
    rt_err_t result;

    RT_UNUSED(user);
    result = hcics_motion_set_target(left, right);
    if (result != RT_EOK)
    {
        hcics_motion_snapshot_t snap;

        hcics_motion_get_snapshot(&snap);
        hcics_latch_motion_fault((snap.stop_reason != 0U) ? snap.stop_reason : 0xE0U);
    }
}

static void hcics_track_get_speed(rt_int16_t *left, rt_int16_t *right, void *user)
{
    rt_int16_t target_left;
    rt_int16_t target_right;

    RT_UNUSED(user);
    hcics_motion_get_realtime(&target_left, left, &target_right, right);
}

static void hcics_forward_clean_camera(const hcics_camera_target_t *target)
{
    hcics_cleaner_camera_target_t clean_target;
    rt_err_t result;

    rt_memset(&clean_target, 0, sizeof(clean_target));
    clean_target.mode = target->mode;
    clean_target.x = target->x;
    clean_target.y = target->y;
    clean_target.width = target->width;
    clean_target.type = target->type;
    clean_target.valid = (target->width > 0U) ? RT_TRUE : RT_FALSE;
    clean_target.tick = target->tick;
    result = hcics_cleaner_update_camera_target(&clean_target);
    if (result != RT_EOK)
    {
        LOG_E("clean camera update failed result=%d", result);
    }
}

static void hcics_control_thread(void *parameter)
{
    RT_UNUSED(parameter);

    while (1)
    {
        hcics_track_tick(hcics_now_ms());
        if (hcics_motion_update() != RT_EOK)
        {
            hcics_motion_snapshot_t snap;

            hcics_motion_get_snapshot(&snap);
            hcics_latch_motion_fault((snap.stop_reason != 0U) ? snap.stop_reason : 0xE0U);
        }
        rt_thread_mdelay(HCICS_CONTROL_PERIOD_MS);
    }
}

void hcics_app_update_camera(const hcics_camera_target_t *target)
{
    if (target == RT_NULL)
    {
        return;
    }

    rt_mutex_take(&g_hcics.camera_lock, RT_WAITING_FOREVER);
    if (target->mode == HCICS_CAM_MODE_CLEAN)
    {
        g_hcics.latest_clean = *target;
    }
    else if (target->mode == HCICS_CAM_MODE_TRACK)
    {
        g_hcics.latest_track = *target;
    }
    rt_mutex_release(&g_hcics.camera_lock);

    if (target->mode == HCICS_CAM_MODE_CLEAN)
    {
        hcics_forward_clean_camera(target);
    }
    else if (target->mode == HCICS_CAM_MODE_TRACK)
    {
        hcics_track_update_camera(target);
    }
}

static void hcics_app_update_role_camera(const hcics_camera_target_t *target,
                                         rt_uint8_t expected_mode,
                                         const char *role)
{
    hcics_camera_target_t role_target;

    if (target == RT_NULL)
    {
        return;
    }

    role_target = *target;
    if (role_target.mode != expected_mode)
    {
        LOG_W("%s camera raw mode=%u forced to mode=%u",
              role, role_target.mode, expected_mode);
        role_target.mode = expected_mode;
    }

    hcics_app_update_camera(&role_target);
}

static rt_err_t hcics_enqueue_fault(const hcics_fault_t *fault,
                                    const struct sockaddr_in *peer)
{
    hcics_msg_t msg;
    rt_err_t result;

    if ((fault == RT_NULL) || (g_hcics.mq == RT_NULL))
    {
        return -RT_EINVAL;
    }
    if (!hcics_protocol_fault_is_valid(fault))
    {
        return -RT_EINVAL;
    }
#ifdef RT_USING_SAL
    if (hcics_pending_report_active())
    {
        return -RT_EBUSY;
    }
#endif

    result = hcics_reserve_mission(fault);
    if (result != RT_EOK)
    {
        return result;
    }

    rt_memset(&msg, 0, sizeof(msg));
    msg.type = HCICS_MSG_FAULT;
    msg.fault = *fault;
#ifdef RT_USING_SAL
    if (peer != RT_NULL)
    {
        msg.peer = *peer;
        msg.peer_valid = RT_TRUE;
    }
#else
    RT_UNUSED(peer);
#endif

    result = rt_mq_send(g_hcics.mq, &msg, sizeof(msg));
    if (result != RT_EOK)
    {
        hcics_release_mission();
        LOG_W("fault queue full, id=%u dropped", fault->id);
    }

    return result;
}

void hcics_app_stop(rt_uint8_t reason)
{
    g_hcics.abort = RT_TRUE;
    hcics_track_stop();
    hcics_cleaner_emergency_stop();
    hcics_motion_emergency_stop(reason);

    LOG_W("safety stop requested reason=%u", reason);
}

static const char *hcics_report_text(hcics_remote_cmd_t cmd)
{
    return (cmd == HCICS_REMOTE_DONE) ? "DONE" : "ERROR";
}

#ifdef RT_USING_SAL
static rt_bool_t hcics_udp_send_report(int sock,
                                       hcics_remote_cmd_t cmd,
                                       rt_uint8_t param,
                                       const struct sockaddr_in *peer)
{
    char text[24];
    int text_len;
    int sent;

    if ((sock < 0) || (peer == RT_NULL))
    {
        return RT_FALSE;
    }
    if (!hcics_wifi_link_ready())
    {
        return RT_FALSE;
    }

    text_len = snprintf(text, sizeof(text), "%s,%u\n",
                        hcics_report_text(cmd), param);
    if ((text_len <= 0) || (text_len >= (int)sizeof(text)))
    {
        return RT_FALSE;
    }

    sent = sendto(sock, text, text_len, 0,
                  (const struct sockaddr *)peer, sizeof(*peer));
    return (sent == text_len) ? RT_TRUE : RT_FALSE;
}

static rt_bool_t hcics_same_peer(const struct sockaddr_in *left,
                                 const struct sockaddr_in *right)
{
    if ((left == RT_NULL) || (right == RT_NULL))
    {
        return RT_FALSE;
    }

    return ((left->sin_family == right->sin_family) &&
            (left->sin_port == right->sin_port) &&
            (left->sin_addr.s_addr == right->sin_addr.s_addr)) ? RT_TRUE : RT_FALSE;
}

static rt_bool_t hcics_store_pending_report_locked(hcics_remote_cmd_t cmd,
                                                   rt_uint8_t param,
                                                   const struct sockaddr_in *peer)
{
    if (peer == RT_NULL)
    {
        return RT_FALSE;
    }
    if (g_hcics.pending_report_valid)
    {
        return RT_FALSE;
    }

    g_hcics.pending_report_cmd = cmd;
    g_hcics.pending_report_param = param;
    g_hcics.pending_report_peer = *peer;
    g_hcics.pending_report_valid = RT_TRUE;

    return RT_TRUE;
}

static void hcics_clear_pending_report_if_current(hcics_remote_cmd_t cmd,
                                                  rt_uint8_t param,
                                                  const struct sockaddr_in *peer)
{
    rt_mutex_take(&g_hcics.udp_lock, RT_WAITING_FOREVER);
    if (g_hcics.pending_report_valid &&
        (g_hcics.pending_report_cmd == cmd) &&
        (g_hcics.pending_report_param == param) &&
        hcics_same_peer(&g_hcics.pending_report_peer, peer))
    {
        g_hcics.pending_report_valid = RT_FALSE;
    }
    rt_mutex_release(&g_hcics.udp_lock);
}

static void hcics_flush_pending_report(void)
{
    hcics_remote_cmd_t cmd;
    rt_uint8_t param;
    struct sockaddr_in peer;
    int sock;

    rt_mutex_take(&g_hcics.udp_lock, RT_WAITING_FOREVER);
    if (!g_hcics.pending_report_valid || (g_hcics.udp_sock < 0) ||
        !hcics_wifi_link_ready())
    {
        rt_mutex_release(&g_hcics.udp_lock);
        return;
    }

    cmd = g_hcics.pending_report_cmd;
    param = g_hcics.pending_report_param;
    peer = g_hcics.pending_report_peer;
    sock = g_hcics.udp_sock;
    rt_mutex_release(&g_hcics.udp_lock);

    if (hcics_udp_send_report(sock, cmd, param, &peer))
    {
        hcics_clear_pending_report_if_current(cmd, param, &peer);
        LOG_I("pending report sent %s,%u",
              hcics_report_text(cmd), param);
    }
}

static rt_bool_t hcics_pending_report_active(void)
{
    rt_bool_t active;

    rt_mutex_take(&g_hcics.udp_lock, RT_WAITING_FOREVER);
    active = g_hcics.pending_report_valid;
    rt_mutex_release(&g_hcics.udp_lock);

    return active;
}
#endif

static void hcics_report_esp_final(hcics_remote_cmd_t cmd,
                                   rt_uint8_t param,
                                   const struct sockaddr_in *peer)
{
#ifdef RT_USING_SAL
    if (peer != RT_NULL)
    {
        struct sockaddr_in peer_copy;
        int sock;
        rt_bool_t sent = RT_FALSE;
        rt_bool_t pending = RT_FALSE;

        rt_mutex_take(&g_hcics.udp_lock, RT_WAITING_FOREVER);
        peer_copy = *peer;
        sock = g_hcics.udp_sock;
        rt_mutex_release(&g_hcics.udp_lock);

        sent = hcics_udp_send_report(sock, cmd, param, &peer_copy);
        if (!sent)
        {
            rt_mutex_take(&g_hcics.udp_lock, RT_WAITING_FOREVER);
            pending = hcics_store_pending_report_locked(cmd, param, &peer_copy);
            rt_mutex_release(&g_hcics.udp_lock);
            if (pending)
            {
                LOG_W("report pending %s,%u", hcics_report_text(cmd), param);
            }
            else
            {
                LOG_E("pending report occupied, keep older result before %s,%u",
                      hcics_report_text(cmd), param);
            }
        }
    }
#else
    RT_UNUSED(peer);
#endif

    LOG_I("report cmd=%d param=%u", cmd, param);
}

#ifdef RT_USING_SAL
static void hcics_report_esp_reject(hcics_remote_cmd_t cmd,
                                    rt_uint8_t param,
                                    const struct sockaddr_in *peer)
{
    rt_bool_t sent = RT_FALSE;

    if (peer != RT_NULL)
    {
        int sock;

        rt_mutex_take(&g_hcics.udp_lock, RT_WAITING_FOREVER);
        sock = g_hcics.udp_sock;
        rt_mutex_release(&g_hcics.udp_lock);

        sent = hcics_udp_send_report(sock, cmd, param, peer);
    }

    LOG_W("reject report cmd=%d param=%u sent=%u",
          cmd, param, sent ? 1U : 0U);
}
#endif

static rt_err_t hcics_open_camera_uart(const char *uart_name,
                                       rt_device_t *uart,
                                       const char *role)
{
    rt_err_t result;

    if ((uart_name == RT_NULL) || (uart == RT_NULL))
    {
        return -RT_EINVAL;
    }

    *uart = rt_device_find(uart_name);
    if (*uart == RT_NULL)
    {
        LOG_E("%s camera uart %s not found", role, uart_name);
        return -RT_ENOSYS;
    }

    result = rt_device_open(*uart,
                            RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    if (result != RT_EOK)
    {
        LOG_E("%s camera uart %s open failed result=%d",
              role, uart_name, result);
        *uart = RT_NULL;
        return result;
    }

    return RT_EOK;
}

static rt_err_t hcics_hw_init(void)
{
    rt_err_t result;

    result = hcics_open_camera_uart(HCICS_TRACK_CAMERA_UART_NAME,
                                    &g_hcics.track_camera_uart,
                                    "track");
    if (result != RT_EOK)
    {
        return result;
    }

    result = hcics_open_camera_uart(HCICS_CLEAN_CAMERA_UART_NAME,
                                    &g_hcics.clean_camera_uart,
                                    "clean");
    if (result != RT_EOK)
    {
        (void)rt_device_close(g_hcics.track_camera_uart);
        g_hcics.track_camera_uart = RT_NULL;
        return result;
    }

    return RT_EOK;
}

static rt_bool_t hcics_wifi_link_ready(void)
{
#ifdef RT_USING_WIFI
    return rt_wlan_is_ready();
#else
    return RT_FALSE;
#endif
}

static rt_err_t hcics_wifi_wait_ready(void)
{
#ifdef RT_USING_WIFI
    rt_err_t result;
    rt_uint32_t start_ms;
    rt_bool_t connect_requested = RT_FALSE;

    result = rt_hw_wlan_wait_init_done(HCICS_WIFI_INIT_TIMEOUT_MS);
    if (result != RT_EOK)
    {
        LOG_E("wifi init timeout, result=%d", result);
        return result;
    }

    rt_wlan_config_autoreconnect(RT_TRUE);
    start_ms = hcics_now_ms();

    while ((hcics_now_ms() - start_ms) < HCICS_WIFI_CONNECT_TIMEOUT_MS)
    {
        if (rt_wlan_is_ready())
        {
            LOG_I("wifi ready ssid=%s", HCICS_WIFI_SSID);
            return RT_EOK;
        }

        if (!rt_wlan_is_connected() && !connect_requested)
        {
            LOG_I("wifi connect ssid=%s", HCICS_WIFI_SSID);
            result = rt_wlan_connect(HCICS_WIFI_SSID, HCICS_WIFI_PASSWORD);
            if (result != RT_EOK)
            {
                LOG_W("wifi connect request failed result=%d", result);
            }
            connect_requested = RT_TRUE;
        }

        rt_thread_mdelay(HCICS_WIFI_POLL_MS);
    }

    LOG_E("wifi not ready after %u ms", HCICS_WIFI_CONNECT_TIMEOUT_MS);
    return -RT_ETIMEOUT;
#else
    LOG_E("RT_USING_WIFI is disabled; UDP link requires AP6212 STA");
    return -RT_ENOSYS;
#endif
}

static rt_uint8_t hcics_fault_to_track_target(const hcics_fault_t *fault)
{
    if (fault == RT_NULL)
    {
        return HCICS_TRACK_TARGET_HOME;
    }

    if (hcics_protocol_fault_is_valid(fault) &&
        (fault->id >= HCICS_TRACK_TARGET_MIN) &&
        (fault->id <= HCICS_TRACK_TARGET_MAX))
    {
        return fault->id;
    }

    return HCICS_TRACK_TARGET_HOME;
}

static rt_err_t hcics_follow_track_path(rt_uint8_t target_id, const char *name)
{
    rt_err_t result;
    rt_uint32_t start_ms;

    result = hcics_wait_track_camera(HCICS_TRACK_WAIT_CAMERA_MS);
    if (result != RT_EOK)
    {
        return result;
    }

    result = hcics_track_start_path(target_id);
    if (result != RT_EOK)
    {
        LOG_E("track path start failed target=%u result=%d", target_id, result);
        return result;
    }

    start_ms = hcics_now_ms();
    LOG_I("track path=%s target=%u", name, target_id);

    while (!g_hcics.abort)
    {
        rt_uint32_t now_ms = hcics_now_ms();

        if (g_hcics.motion_fault)
        {
            hcics_track_stop();
            (void)hcics_motion_set_target_direct(0, 0);
            LOG_E("track aborted by motion fault reason=%u",
                  g_hcics.motion_fault_reason);
            return -RT_ERROR;
        }

        if (hcics_track_is_finished())
        {
            (void)hcics_motion_set_target_direct(0, 0);
            return RT_EOK;
        }

        {
            hcics_track_status_t status;

            hcics_track_status(&status);
            if (hcics_track_state_requires_camera(status.visual_mission_state) &&
                !hcics_latest_track_recent(HCICS_TRACK_LOST_CAMERA_MS))
            {
                hcics_track_stop();
                (void)hcics_motion_set_target_direct(0, 0);
                LOG_E("track camera lost path=%s target=%u state=%s",
                      name, target_id,
                      hcics_track_state_name(status.visual_mission_state));
                return -RT_ETIMEOUT;
            }
        }

        if ((now_ms - start_ms) >= HCICS_TRACK_NAV_TIMEOUT_MS)
        {
            hcics_track_stop();
            (void)hcics_motion_set_target_direct(0, 0);
            LOG_E("track path timeout target=%u", target_id);
            return -RT_ETIMEOUT;
        }

        rt_thread_mdelay(HCICS_TRACK_POLL_MS);
    }

    hcics_track_stop();
    (void)hcics_motion_set_target_direct(0, 0);
    if (g_hcics.motion_fault)
    {
        LOG_E("track aborted by motion fault reason=%u",
              g_hcics.motion_fault_reason);
        return -RT_ERROR;
    }
    return -RT_EINTR;
}

static rt_err_t hcics_navigate_to_fault(const hcics_fault_t *fault)
{
    rt_uint8_t target_id;

    if (fault == RT_NULL)
    {
        return -RT_EINVAL;
    }

    target_id = hcics_fault_to_track_target(fault);
    if (target_id == HCICS_TRACK_TARGET_HOME)
    {
        LOG_E("invalid fault target id=%u row=%u col=%u",
              fault->id, fault->row, fault->col);
        return -RT_EINVAL;
    }

    LOG_I("navigate fault id=%u row=%u col=%u severity=%u source=%s target=%u",
          fault->id, fault->row, fault->col, fault->severity, fault->source,
          target_id);

    return hcics_follow_track_path(target_id, "nav");
}

static rt_err_t hcics_return_home(const hcics_fault_t *fault)
{
    if (fault == RT_NULL)
    {
        return -RT_EINVAL;
    }

    LOG_I("return home from id=%u", fault->id);
    return hcics_follow_track_path(HCICS_TRACK_TARGET_HOME, "return");
}

static rt_err_t hcics_clean_panel(void)
{
    rt_err_t result;
    rt_uint32_t start_ms;

    result = hcics_cleaner_start(HCICS_CLEAN_MAX_CYCLES);
    if (result != RT_EOK)
    {
        LOG_E("cleaner start failed result=%d", result);
        return result;
    }

    start_ms = hcics_now_ms();
    while (!g_hcics.abort)
    {
        hcics_cleaner_status_t status;

        result = hcics_cleaner_get_status(&status);
        if (result != RT_EOK)
        {
            return result;
        }

        if (status.state == HCICS_CLEANER_STATE_DONE)
        {
            LOG_I("cleaner done cycles=%u result=%d", status.cycle, status.last_result);
            return status.last_result;
        }
        if (status.state == HCICS_CLEANER_STATE_ERROR)
        {
            LOG_E("cleaner error result=%d cycle=%u", status.last_result, status.cycle);
            return (status.last_result == RT_EOK) ? -RT_ERROR : status.last_result;
        }
        if (status.state == HCICS_CLEANER_STATE_STOPPED)
        {
            return -RT_EINTR;
        }

        if ((hcics_now_ms() - start_ms) >= HCICS_CLEAN_TIMEOUT_MS)
        {
            hcics_cleaner_emergency_stop();
            LOG_E("cleaner timeout");
            return -RT_ETIMEOUT;
        }

        rt_thread_mdelay(HCICS_CLEAN_POLL_MS);
    }

    hcics_cleaner_emergency_stop();
    return -RT_EINTR;
}

static void hcics_safe_stop_outputs(void)
{
    hcics_track_stop();
    hcics_cleaner_reset();
    (void)hcics_motion_set_target_direct(0, 0);
}

static rt_err_t hcics_run_mission(const hcics_msg_t *msg)
{
    rt_err_t result;
    const hcics_fault_t *fault;
#ifdef RT_USING_SAL
    const struct sockaddr_in *peer;
#endif

    if ((msg == RT_NULL) || !hcics_protocol_fault_is_valid(&msg->fault))
    {
        return -RT_EINVAL;
    }

    fault = &msg->fault;
#ifdef RT_USING_SAL
    peer = msg->peer_valid ? &msg->peer : RT_NULL;
#endif

    g_hcics.abort = RT_FALSE;
    g_hcics.motion_fault = RT_FALSE;
    g_hcics.motion_fault_reason = 0U;
    hcics_motion_clear_emergency_stop();
    hcics_motion_reset();
    {
        hcics_motion_snapshot_t snap;

        hcics_motion_get_snapshot(&snap);
        if (snap.emergency_stop)
        {
            g_hcics.motion_fault = RT_TRUE;
            g_hcics.motion_fault_reason = (snap.stop_reason != 0U) ? snap.stop_reason : 0xE0U;
            result = -RT_ERROR;
            LOG_E("motion reset failed before mission reason=%u",
                  g_hcics.motion_fault_reason);
            goto mission_failed;
        }
    }
    hcics_cleaner_reset();

    hcics_set_state(HCICS_STATE_NAV_TO_FAULT);
    result = hcics_navigate_to_fault(fault);
    if (result != RT_EOK)
    {
        goto mission_failed;
    }

    hcics_set_state(HCICS_STATE_PARKED);
    if (!hcics_delay_abortable(500U))
    {
        result = -RT_EINTR;
        goto mission_failed;
    }

    hcics_set_state(HCICS_STATE_CLEANING);
    result = hcics_clean_panel();
    if (result != RT_EOK)
    {
        goto mission_failed;
    }

    hcics_set_state(HCICS_STATE_RETURN_HOME);
    result = hcics_return_home(fault);
    if (result != RT_EOK)
    {
        goto mission_failed;
    }

    hcics_safe_stop_outputs();
    hcics_set_state(HCICS_STATE_IDLE);
    hcics_report_esp_final(HCICS_REMOTE_DONE, fault->id, peer);
    hcics_release_mission();
    return RT_EOK;

mission_failed:
    hcics_safe_stop_outputs();
    hcics_set_state(HCICS_STATE_ERROR);
    hcics_report_esp_final(HCICS_REMOTE_ERROR, fault->id, peer);
    hcics_release_mission();
    LOG_E("mission failed result=%d", result);
    hcics_set_state(HCICS_STATE_IDLE);
    return result;
}

static void hcics_mission_thread(void *parameter)
{
    hcics_msg_t msg;

    (void)parameter;

    while (1)
    {
        if (rt_mq_recv(g_hcics.mq, &msg, sizeof(msg), RT_WAITING_FOREVER) != RT_EOK)
        {
            continue;
        }

        if (msg.type == HCICS_MSG_FAULT)
        {
            (void)hcics_run_mission(&msg);
        }
    }
}

static void hcics_camera_uart_loop(rt_device_t camera_uart,
                                   const char *uart_name,
                                   rt_uint8_t expected_mode,
                                   const char *role)
{
    rt_uint8_t byte;
    hcics_camera_parser_t parser;

    hcics_protocol_camera_parser_init(&parser);

    if (camera_uart == RT_NULL)
    {
        LOG_E("%s camera uart %s is not opened", role, uart_name);
        return;
    }

    LOG_I("%s camera uart %s ready", role, uart_name);

    while (1)
    {
        if (rt_device_read(camera_uart, 0, &byte, 1) == 1)
        {
            hcics_camera_target_t target;

            if (hcics_protocol_camera_feed(&parser, byte, &target))
            {
                hcics_app_update_role_camera(&target, expected_mode, role);
                LOG_D("%s cam raw_mode=%u role_mode=%u x=%u y=%u w=%u type=%u",
                      role, target.mode, expected_mode, target.x, target.y,
                      target.width, target.type);
            }
        }
        else
        {
            rt_thread_mdelay(10);
        }
    }
}

static void hcics_track_camera_uart_thread(void *parameter)
{
    (void)parameter;
    hcics_camera_uart_loop(g_hcics.track_camera_uart,
                           HCICS_TRACK_CAMERA_UART_NAME,
                           HCICS_CAM_MODE_TRACK,
                           "track");
}

static void hcics_clean_camera_uart_thread(void *parameter)
{
    (void)parameter;
    hcics_camera_uart_loop(g_hcics.clean_camera_uart,
                           HCICS_CLEAN_CAMERA_UART_NAME,
                           HCICS_CAM_MODE_CLEAN,
                           "clean");
}

#ifdef RT_USING_SAL
static void hcics_udp_thread(void *parameter)
{
    (void)parameter;

    while (1)
    {
        int sock;
        struct sockaddr_in local;

        if (hcics_wifi_wait_ready() != RT_EOK)
        {
            rt_thread_mdelay(HCICS_WIFI_RETRY_DELAY_MS);
            continue;
        }

        sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0)
        {
            LOG_E("udp socket create failed");
            rt_thread_mdelay(HCICS_WIFI_RETRY_DELAY_MS);
            continue;
        }

#ifdef SO_RCVTIMEO
        {
            struct timeval timeout;

            timeout.tv_sec = 1;
            timeout.tv_usec = 0;
            (void)setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO,
                             &timeout, sizeof(timeout));
        }
#endif

        rt_memset(&local, 0, sizeof(local));
        local.sin_family = AF_INET;
        local.sin_port = htons(HCICS_NET_PORT);
        local.sin_addr.s_addr = htonl(INADDR_ANY);

        if (bind(sock, (struct sockaddr *)&local, sizeof(local)) < 0)
        {
            LOG_E("udp bind port=%u failed", HCICS_NET_PORT);

            rt_mutex_take(&g_hcics.udp_lock, RT_WAITING_FOREVER);
            g_hcics.udp_sock = -1;
            rt_mutex_release(&g_hcics.udp_lock);

            closesocket(sock);
            rt_thread_mdelay(HCICS_WIFI_RETRY_DELAY_MS);
            continue;
        }

        rt_mutex_take(&g_hcics.udp_lock, RT_WAITING_FOREVER);
        g_hcics.udp_sock = sock;
        rt_mutex_release(&g_hcics.udp_lock);

        LOG_I("udp command server ready port=%u", HCICS_NET_PORT);
        hcics_flush_pending_report();

        while (1)
        {
            char buf[128];
            struct sockaddr_in from;
            socklen_t from_len = sizeof(from);
            hcics_fault_t fault;
            int len;
            int i;
            rt_bool_t has_embedded_nul = RT_FALSE;

            if (!hcics_wifi_link_ready())
            {
                LOG_W("wifi link lost; restart UDP command server");
                break;
            }

            len = recvfrom(sock, buf, sizeof(buf) - 1, 0,
                           (struct sockaddr *)&from, &from_len);
            if (len <= 0)
            {
                hcics_flush_pending_report();
                continue;
            }

            for (i = 0; i < len; i++)
            {
                if (buf[i] == '\0')
                {
                    has_embedded_nul = RT_TRUE;
                    break;
                }
            }
            if (has_embedded_nul)
            {
                LOG_W("udp packet rejected: embedded NUL len=%d", len);
                continue;
            }

            buf[len] = '\0';
            hcics_flush_pending_report();
            if (hcics_protocol_parse_fault_text(buf, HCICS_UDP_TOKEN, &fault))
            {
                if (hcics_pending_report_active())
                {
                    LOG_W("pending report blocks new fault id=%u", fault.id);
                    hcics_report_esp_reject(HCICS_REMOTE_ERROR, fault.id, &from);
                    continue;
                }

                len = hcics_enqueue_fault(&fault, &from);
                if (len == -RT_EBUSY)
                {
                    LOG_W("mission busy, reject fault id=%u", fault.id);
                    hcics_report_esp_reject(HCICS_REMOTE_ERROR, fault.id, &from);
                    continue;
                }
                if (len != RT_EOK)
                {
                    LOG_W("fault enqueue failed id=%u result=%d", fault.id, len);
                    hcics_report_esp_reject(HCICS_REMOTE_ERROR, fault.id, &from);
                }
            }
            else if (hcics_protocol_parse_stop_text(buf, HCICS_UDP_TOKEN))
            {
                hcics_app_stop(0xDDU);
            }
            else
            {
                LOG_W("udp unknown: %s", buf);
            }
        }

        rt_mutex_take(&g_hcics.udp_lock, RT_WAITING_FOREVER);
        g_hcics.udp_sock = -1;
        rt_mutex_release(&g_hcics.udp_lock);
        closesocket(sock);
        rt_thread_mdelay(HCICS_WIFI_RETRY_DELAY_MS);
    }
}
#else
static void hcics_udp_thread(void *parameter)
{
    (void)parameter;
    LOG_W("RT_USING_SAL is disabled; UDP command server is not started");
}
#endif

int hcics_app_init(void)
{
    rt_thread_t control_tid;
    rt_thread_t mission_tid;
    rt_thread_t udp_tid;
    rt_thread_t track_camera_tid;
    rt_thread_t clean_camera_tid;
    hcics_track_motion_ops_t track_ops;
    rt_err_t result;

    if (g_hcics.initialized)
    {
        return RT_EOK;
    }

    rt_memset(&g_hcics, 0, sizeof(g_hcics));
    g_hcics.state = HCICS_STATE_IDLE;
#ifdef RT_USING_SAL
    g_hcics.udp_sock = -1;
#endif

    rt_mutex_init(&g_hcics.camera_lock, "hcics_cam", RT_IPC_FLAG_FIFO);
    rt_mutex_init(&g_hcics.state_lock, "hcics_st", RT_IPC_FLAG_FIFO);
#ifdef RT_USING_SAL
    rt_mutex_init(&g_hcics.udp_lock, "hcics_udp", RT_IPC_FLAG_FIFO);
#endif
    g_hcics.mq = rt_mq_create("hcics_mq", sizeof(hcics_msg_t), HCICS_EVENT_DEPTH, RT_IPC_FLAG_FIFO);
    if (g_hcics.mq == RT_NULL)
    {
        LOG_E("message queue create failed");
        return -RT_ENOMEM;
    }

    result = hcics_hw_init();
    if (result != RT_EOK)
    {
        LOG_E("hardware init failed result=%d", result);
        return result;
    }

    result = hcics_motion_init();
    if (result != RT_EOK)
    {
        LOG_E("motion init failed result=%d", result);
        return result;
    }

    result = hcics_axis_init();
    if (result != RT_EOK)
    {
        LOG_E("axis init failed result=%d", result);
        return result;
    }

    rt_memset(&track_ops, 0, sizeof(track_ops));
    track_ops.set_motor = hcics_track_set_motor;
    track_ops.get_speed = hcics_track_get_speed;
    result = hcics_track_init(&track_ops);
    if (result != RT_EOK)
    {
        LOG_E("track init failed result=%d", result);
        return result;
    }

    result = hcics_cleaner_init(RT_NULL);
    if (result != RT_EOK)
    {
        LOG_E("cleaner init failed result=%d", result);
        return result;
    }

    result = hcics_guard_init();
    if (result != RT_EOK)
    {
        LOG_E("guard init failed result=%d", result);
        return result;
    }

    control_tid = rt_thread_create("hcics_ctl", hcics_control_thread, RT_NULL,
                                   HCICS_CONTROL_THREAD_STACK,
                                   HCICS_CONTROL_THREAD_PRIO, 2);
    mission_tid = rt_thread_create("hcics", hcics_mission_thread, RT_NULL,
                                   HCICS_THREAD_STACK, HCICS_THREAD_PRIO, 20);
    udp_tid = rt_thread_create("hcics_udp", hcics_udp_thread, RT_NULL,
                               HCICS_IO_THREAD_STACK, HCICS_IO_THREAD_PRIO, 20);
    track_camera_tid = rt_thread_create("cam_trk", hcics_track_camera_uart_thread, RT_NULL,
                                        HCICS_IO_THREAD_STACK, HCICS_IO_THREAD_PRIO, 20);
    clean_camera_tid = rt_thread_create("cam_cln", hcics_clean_camera_uart_thread, RT_NULL,
                                        HCICS_IO_THREAD_STACK, HCICS_IO_THREAD_PRIO, 20);
    if ((control_tid == RT_NULL) || (mission_tid == RT_NULL) ||
        (udp_tid == RT_NULL) || (track_camera_tid == RT_NULL) ||
        (clean_camera_tid == RT_NULL))
    {
        LOG_E("thread create failed ctl=%p mission=%p udp=%p tcam=%p ccam=%p",
              control_tid, mission_tid, udp_tid,
              track_camera_tid, clean_camera_tid);
        if (control_tid != RT_NULL)
        {
            (void)rt_thread_delete(control_tid);
        }
        if (mission_tid != RT_NULL)
        {
            (void)rt_thread_delete(mission_tid);
        }
        if (udp_tid != RT_NULL)
        {
            (void)rt_thread_delete(udp_tid);
        }
        if (track_camera_tid != RT_NULL)
        {
            (void)rt_thread_delete(track_camera_tid);
        }
        if (clean_camera_tid != RT_NULL)
        {
            (void)rt_thread_delete(clean_camera_tid);
        }
        return -RT_ENOMEM;
    }

    rt_thread_startup(control_tid);
    rt_thread_startup(mission_tid);
    rt_thread_startup(udp_tid);
    rt_thread_startup(track_camera_tid);
    rt_thread_startup(clean_camera_tid);

    g_hcics.initialized = RT_TRUE;
    LOG_I("HCICS app initialized");
    return RT_EOK;
}
