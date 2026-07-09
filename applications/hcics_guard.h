/*
 * HCICS cleaner guard input.
 *
 * The guard feeds real distance/contact data into the cleaner module.  It is
 * intentionally a required production device: if the configured sensor bus or
 * sensor is absent, initialization fails instead of faking a safe surface.
 */

#ifndef HCICS_GUARD_H
#define HCICS_GUARD_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

rt_err_t hcics_guard_init(void);
rt_bool_t hcics_guard_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* HCICS_GUARD_H */
