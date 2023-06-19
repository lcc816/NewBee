/*******************************************************************************
 * @file     motor.h
 * @author   lcc
 * @version
 * @date     2022-10-04
 * @brief
 ******************************************************************************/
#ifndef __MOTOR_H_
#define __MOTOR_H_

struct motor_device
{
    struct rt_device_pwm *pwm_dev;
    int channel;
    rt_uint32_t period;
    rt_uint16_t max_level;
};

rt_err_t motor_init(struct motor_device *dev, const char *pwm_name, int channel,
                    rt_uint32_t period, rt_uint16_t max_level);

void motor_set_speed(struct motor_device *dev, uint16_t level);

#endif /* __MOTOR_H_ */
