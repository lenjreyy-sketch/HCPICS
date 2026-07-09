/*
 * HCICS protocol helpers.
 */

#include "hcics_protocol.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define HCICS_FAULT_GRID_COLS    3
#define HCICS_FAULT_TARGET_MIN   1
#define HCICS_FAULT_TARGET_MAX   9

static rt_bool_t hcics_fault_id_to_grid(int id, int *row, int *col)
{
    if ((id < HCICS_FAULT_TARGET_MIN) || (id > HCICS_FAULT_TARGET_MAX))
    {
        return RT_FALSE;
    }

    if (row != RT_NULL)
    {
        *row = ((id - 1) / HCICS_FAULT_GRID_COLS) + 1;
    }
    if (col != RT_NULL)
    {
        *col = ((id - 1) % HCICS_FAULT_GRID_COLS) + 1;
    }

    return RT_TRUE;
}

static rt_bool_t hcics_fault_grid_to_id(int row, int col, int *id)
{
    if ((row < 1) || (row > 3) || (col < 1) || (col > 3))
    {
        return RT_FALSE;
    }

    if (id != RT_NULL)
    {
        *id = ((row - 1) * HCICS_FAULT_GRID_COLS) + col;
    }

    return RT_TRUE;
}

static void hcics_fault_defaults(hcics_fault_t *fault)
{
    if (fault == RT_NULL)
    {
        return;
    }

    fault->id = 1U;
    fault->row = 1U;
    fault->col = 1U;
    fault->severity = 1U;
    rt_strncpy(fault->source, "udp", sizeof(fault->source) - 1U);
    fault->source[sizeof(fault->source) - 1U] = '\0';
}

static rt_bool_t hcics_parse_uint_field(const char **cursor,
                                        char delimiter,
                                        int *value)
{
    const char *p;
    int parsed = 0;

    if ((cursor == RT_NULL) || (*cursor == RT_NULL) || (value == RT_NULL))
    {
        return RT_FALSE;
    }

    p = *cursor;
    if ((*p < '0') || (*p > '9'))
    {
        return RT_FALSE;
    }

    while ((*p >= '0') && (*p <= '9'))
    {
        parsed = (parsed * 10) + (*p - '0');
        if (parsed > 255)
        {
            return RT_FALSE;
        }

        p++;
    }

    if (delimiter != '\0')
    {
        if (*p != delimiter)
        {
            return RT_FALSE;
        }
        p++;
    }
    else
    {
        while ((*p == '\r') || (*p == '\n') || (*p == ' ') || (*p == '\t'))
        {
            p++;
        }
        if (*p != '\0')
        {
            return RT_FALSE;
        }
    }

    *cursor = p;
    *value = parsed;
    return RT_TRUE;
}

static rt_bool_t hcics_token_is_usable(const char *token)
{
    const char *p;

    if ((token == RT_NULL) || (*token == '\0'))
    {
        return RT_FALSE;
    }

    for (p = token; *p != '\0'; p++)
    {
        if ((*p == ',') || (*p == '\r') || (*p == '\n') ||
            (*p == ' ') || (*p == '\t'))
        {
            return RT_FALSE;
        }
    }

    return RT_TRUE;
}

static rt_bool_t hcics_parse_token_field(const char **cursor,
                                         char delimiter,
                                         const char *token)
{
    const char *p;
    const char *t;

    if ((cursor == RT_NULL) || (*cursor == RT_NULL) ||
        !hcics_token_is_usable(token))
    {
        return RT_FALSE;
    }

    p = *cursor;
    t = token;
    while ((*t != '\0') && (*p == *t))
    {
        p++;
        t++;
    }

    if (*t != '\0')
    {
        return RT_FALSE;
    }

    if (delimiter != '\0')
    {
        if (*p != delimiter)
        {
            return RT_FALSE;
        }
        p++;
    }
    else
    {
        while ((*p == '\r') || (*p == '\n') || (*p == ' ') || (*p == '\t'))
        {
            p++;
        }
        if (*p != '\0')
        {
            return RT_FALSE;
        }
    }

    *cursor = p;
    return RT_TRUE;
}

void hcics_protocol_camera_parser_init(hcics_camera_parser_t *parser)
{
    if (parser == RT_NULL)
    {
        return;
    }

    rt_memset(parser, 0, sizeof(*parser));
}

rt_bool_t hcics_protocol_camera_feed(hcics_camera_parser_t *parser,
                                     rt_uint8_t byte,
                                     hcics_camera_target_t *target)
{
    if ((parser == RT_NULL) || (target == RT_NULL))
    {
        return RT_FALSE;
    }

    if (parser->state == 0U)
    {
        if (byte == HCICS_CAM_HEAD)
        {
            parser->buf[0] = byte;
            parser->index = 1U;
            parser->state = 1U;
        }
        return RT_FALSE;
    }

    parser->buf[parser->index++] = byte;
    if (parser->index < HCICS_CAM_FRAME_LEN)
    {
        return RT_FALSE;
    }

    parser->state = 0U;
    parser->index = 0U;

    if (parser->buf[8] != HCICS_CAM_TAIL)
    {
        return RT_FALSE;
    }

    target->mode = parser->buf[1];
    target->x = (rt_uint16_t)(((rt_uint16_t)parser->buf[2] << 8) | parser->buf[3]);
    target->y = (rt_uint16_t)(((rt_uint16_t)parser->buf[4] << 8) | parser->buf[5]);
    target->width = parser->buf[6];
    target->type = parser->buf[7];
    target->tick = rt_tick_get();

    return ((target->mode == HCICS_CAM_MODE_TRACK) ||
            (target->mode == HCICS_CAM_MODE_CLEAN)) ? RT_TRUE : RT_FALSE;
}

rt_bool_t hcics_protocol_fault_is_valid(const hcics_fault_t *fault)
{
    int expected_row;
    int expected_col;

    if (fault == RT_NULL)
    {
        return RT_FALSE;
    }

    if (!hcics_fault_id_to_grid(fault->id, &expected_row, &expected_col))
    {
        return RT_FALSE;
    }

    if ((fault->row != (rt_uint8_t)expected_row) ||
        (fault->col != (rt_uint8_t)expected_col) ||
        (fault->severity == 0U))
    {
        return RT_FALSE;
    }

    return RT_TRUE;
}

rt_bool_t hcics_protocol_parse_fault_text(const char *text,
                                          const char *token,
                                          hcics_fault_t *fault)
{
    int id = 0;
    int row = 0;
    int col = 0;
    int severity = 1;
    const char *p;

    if ((text == RT_NULL) || (fault == RT_NULL) ||
        !hcics_token_is_usable(token))
    {
        return RT_FALSE;
    }

    hcics_fault_defaults(fault);

    if (rt_strncmp(text, "FAULT,", 6) != 0)
    {
        return RT_FALSE;
    }

    p = text + 6;
    if (!hcics_parse_token_field(&p, ',', token) ||
        !hcics_parse_uint_field(&p, ',', &id) ||
        !hcics_parse_uint_field(&p, ',', &row) ||
        !hcics_parse_uint_field(&p, ',', &col) ||
        !hcics_parse_uint_field(&p, '\0', &severity))
    {
        return RT_FALSE;
    }

    if (!hcics_fault_grid_to_id(row, col, RT_NULL))
    {
        return RT_FALSE;
    }

    fault->id = (rt_uint8_t)id;
    fault->row = (rt_uint8_t)row;
    fault->col = (rt_uint8_t)col;
    fault->severity = (rt_uint8_t)severity;
    rt_strncpy(fault->source, "udp", sizeof(fault->source) - 1U);
    fault->source[sizeof(fault->source) - 1U] = '\0';

    return hcics_protocol_fault_is_valid(fault);
}

rt_bool_t hcics_protocol_parse_stop_text(const char *text,
                                         const char *token)
{
    const char *p;

    if ((text == RT_NULL) || !hcics_token_is_usable(token))
    {
        return RT_FALSE;
    }

    if (rt_strncmp(text, "STOP,", 5) != 0)
    {
        return RT_FALSE;
    }

    p = text + 5;
    return hcics_parse_token_field(&p, '\0', token);
}
