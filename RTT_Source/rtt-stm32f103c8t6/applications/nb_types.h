/*******************************************************************************
 * @file     nb_types.h
 * @author   lcc
 * @version
 * @date     2022-07-06
 * @brief
 ******************************************************************************/

#ifndef __NB_TYPES_H_
#define __NB_TYPES_H_

struct remote_control
{
    int16_t throttle;
    int16_t pitch;
    int16_t roll;
    int16_t yaw;
};

struct remote_key
{
    uint8_t up:1;
    uint8_t down:1;
    uint8_t left:1;
    uint8_t right:1;
};

struct led_ctrl_s
{
    uint16_t led_mode;
    volatile rt_sem_t irq_sem;
};

#endif /* __NB_TYPES_H_ */
