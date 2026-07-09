/*
 * HCICS X-axis motor driver.
 *
 * The ART-Pi P1/P2 profile uses a software UART TX pin for the cleaner
 * X-axis ZDT/Emm42-class motor so both hardware UART pairs can serve cameras.
 */

#ifndef HCICS_AXIS_H
#define HCICS_AXIS_H

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

rt_err_t hcics_axis_init(void);
rt_bool_t hcics_axis_is_ready(void);

rt_err_t hcics_axis_enable(void);
rt_err_t hcics_axis_stop(void);
rt_err_t hcics_axis_move_pulses(rt_bool_t positive,
                                rt_uint16_t speed,
                                rt_uint32_t pulses);
rt_err_t hcics_axis_speed(rt_bool_t positive, rt_uint16_t speed);
rt_err_t hcics_axis_center_from_error(int error_x);
rt_err_t hcics_axis_sweep_step(rt_bool_t positive);
void hcics_axis_emergency_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* HCICS_AXIS_H */
