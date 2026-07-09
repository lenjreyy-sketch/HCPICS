/*
 * HCICS protocol helpers.
 *
 * Production inputs:
 * - UDP fault task: FAULT,<token>,<id>,<row>,<col>,<severity>
 * - UDP safety stop: STOP,<token>
 * - Camera frames: AA mode xh xl yh yl width type 55
 */

#ifndef HCICS_PROTOCOL_H
#define HCICS_PROTOCOL_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HCICS_CAM_FRAME_LEN       9U

#define HCICS_CAM_HEAD            0xAAU
#define HCICS_CAM_TAIL            0x55U
#define HCICS_CAM_MODE_TRACK      0x01U
#define HCICS_CAM_MODE_CLEAN      0x02U

typedef enum
{
    HCICS_REMOTE_DONE = 0,
    HCICS_REMOTE_ERROR
} hcics_remote_cmd_t;

typedef struct
{
    rt_uint8_t id;
    rt_uint8_t row;
    rt_uint8_t col;
    rt_uint8_t severity;
    char source[12];
} hcics_fault_t;

typedef struct
{
    rt_uint8_t mode;
    rt_uint16_t x;
    rt_uint16_t y;
    rt_uint8_t width;
    rt_uint8_t type;
    rt_tick_t tick;
} hcics_camera_target_t;

typedef struct
{
    rt_uint8_t state;
    rt_uint8_t index;
    rt_uint8_t buf[HCICS_CAM_FRAME_LEN];
} hcics_camera_parser_t;

void hcics_protocol_camera_parser_init(hcics_camera_parser_t *parser);
rt_bool_t hcics_protocol_camera_feed(hcics_camera_parser_t *parser,
                                     rt_uint8_t byte,
                                     hcics_camera_target_t *target);

rt_bool_t hcics_protocol_fault_is_valid(const hcics_fault_t *fault);
rt_bool_t hcics_protocol_parse_fault_text(const char *text,
                                          const char *token,
                                          hcics_fault_t *fault);
rt_bool_t hcics_protocol_parse_stop_text(const char *text,
                                         const char *token);

#ifdef __cplusplus
}
#endif

#endif /* HCICS_PROTOCOL_H */
