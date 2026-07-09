/*
 * HCICS application entry.
 */

#ifndef HCICS_APP_H
#define HCICS_APP_H

#include <rtthread.h>
#include "hcics_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

int hcics_app_init(void);
void hcics_app_stop(rt_uint8_t reason);
void hcics_app_update_camera(const hcics_camera_target_t *target);

#ifdef __cplusplus
}
#endif

#endif /* HCICS_APP_H */
