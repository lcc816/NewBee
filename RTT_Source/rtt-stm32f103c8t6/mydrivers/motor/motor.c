/*******************************************************************************
 * @file     motor.c
 * @author   lcc
 * @version
 * @date     2022-10-04
 * @brief
 ******************************************************************************/
#include <rtthread.h>
#include <rtdevice.h>

#include "motor.h"

#define DBG_TAG     "drv.motor"
#define DBG_LVL     3
#include <rtdbg.h>

rt_err_t motor_init(struct motor_device *dev, const char *pwm_name, int channel,
                    rt_uint32_t period, rt_uint16_t max_level)
{
    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(pwm_name != RT_NULL);

    dev->pwm_dev = RT_NULL;

    if (max_level == 0)
    {
        LOG_E("max_level can not be 0!");
        return -RT_ERROR;
    }

    dev->pwm_dev = (struct rt_device_pwm *)rt_device_find(pwm_name);
    if (dev->pwm_dev == RT_NULL)
    {
        LOG_E("can't find %s device!", pwm_name);
        return -RT_ERROR;
    }

    dev->channel = channel;
    dev->period = period;
    dev->max_level = max_level;

    /* 设置PWM周期和脉冲宽度默认值 */
    rt_pwm_set(dev->pwm_dev, dev->channel, dev->period, 0);
    /* 使能设备 */
    rt_pwm_enable(dev->pwm_dev, dev->channel);

    return RT_EOK;
}

void motor_set_speed(struct motor_device *dev, uint16_t level)
{
    rt_uint32_t pulse = 0;          /* PWM脉冲宽度值，单位为纳秒ns */

    if (level >= dev->max_level)
        level = dev->max_level;

    pulse = level * dev->period / dev->max_level;
    rt_pwm_set(dev->pwm_dev, dev->channel, dev->period, pulse);
}
