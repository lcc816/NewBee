/*******************************************************************************
 * @file    app_common.h
 * @author  lcc
 * @version
 * @date    2023-04-12
 * @brief   Global variables or functions
 ******************************************************************************/

#ifndef __APP_COMM_H_
#define __APP_COMM_H_

#include <rtthread.h>
#include "nb_types.h"

#define BIT(n)          (1<<(n))
#define BITS(m, n)      (~(BIT(m)-1) & ((BIT(n) - 1) | BIT(n)))

#define LED_MODE_FLIGHT_LOCK    0
#define LED_MODE_FLIGHT_UNLOCK  1
#define LED_MODE_RUN_ERROR      2
#define LED_MODE_IMU_FAIL       3
#define LED_MODE_COMM_FAIL      4

#define AIRPLANE_STATUS_LOCK    0
#define AIRPLANE_STATUS_UNLOCK  1

extern struct led_ctrl_s led_ctrl;

rt_uint8_t nb_control_status_get(void);
void nb_control_status_set(rt_uint8_t status);

void nb_remote_control_update(struct remote_control *rc);
rt_bool_t nb_remote_comm_ok(void);

#endif /* __APP_COMM_H_ */
