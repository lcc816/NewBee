/*******************************************************************************
 * @file     nb_control_thread.c
 * @author   lcc
 * @version
 * @date     2022-08-03
 * @brief
 ******************************************************************************/

/*
 *                    (Y+)
 *                M1    ↑    M2
 *                  \   |   /
 *                   \  |  /
 *                    \ | /
 *              ————————+————————>X+
 *                    / | \
 *                   /  |  \
 *                  /   |   \
 *                M4    |    M3
 */
#include "nb_common.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "rtt_mpu_adpt.h"
#include "rtt_mpu_config.h"

#include <rtthread.h>
#include <board.h>

#include <stdio.h>
#include <stdint.h>

#include "motor.h"
#include "nb_types.h"
#include "app_common.h"
#include "rich_led.h"

#define DBG_TAG     "app.control"
#define DBG_LVL     2
#include <rtdbg.h>

#define RAD_TO_DEG  57.324841f
#define DEG_TO_RAD  0.0174533f

struct pid
{
    float p;
    float i;
    float d;
    float error;
    float integral;
    float differ;
    float p_err;
    float pp_err;
    float i_lim;
    float i_ran;
    float p_out;
    float i_out;
    float d_out;
    float output;
    uint8_t i_lim_flag;
};

struct pid pid_roll_angle, pid_pitch_angle;
struct pid pid_roll_rate, pid_pitch_rate, pid_yaw_rate;

float moto_pwm_1 = 0.0f, moto_pwm_2 = 0.0f, moto_pwm_3 = 0.0f, moto_pwm_4 = 0.0f;
struct motor_device motor1, motor2, motor3, motor4;

rt_uint8_t g_airplane_status = AIRPLANE_STATUS_LOCK;
rt_bool_t g_imu_failed = false;

//static struct rt_thread control_thread;
//static char control_thread_stack[1024];

void pid_position_cal(struct pid *pid, float target, float measure, rt_bool_t int_on)
{
    pid->error = target - measure; // 当前角度误差
    pid->differ = pid->error - pid->p_err; // 

    pid->p_out = pid->p * pid->error; // 单环 P 项输出
    pid->i_out = pid->i_lim_flag * pid->i * pid->integral;
    pid->d_out = pid->d * pid->differ;

    pid->output = pid->p_out + pid->i_out + pid->d_out;

    if (int_on)
    {
        if (measure > (pid->i_lim) || measure < -pid->i_lim)
        {
            pid->i_lim_flag = 0;
        }
        else
        {
            /* 加入积分, 只有测量值在 [-Ilimit,Ilimit] 范围内时才加入积分 */
            pid->i_lim_flag = 1;
            /* 对误差进行积分 */
            pid->integral += pid->error;
            /* 积分限幅 */
            if (pid->integral > pid->i_ran)
            {
                pid->integral = pid->i_ran;
            }
            if (pid->integral < -pid->i_ran)
            {
                pid->integral = -pid->i_ran;
            }
        }
    }
    else
    {
        pid->integral = 0;
    }

    pid->p_err = pid->error;
}

rt_uint8_t nb_control_status_get(void)
{
    return g_airplane_status;
}

void nb_control_status_set(rt_uint8_t status)
{
    g_airplane_status = status;
}

void control(struct euler_float *att_in, struct vector3_float *gyros_in,
             struct remote_control *rc_in, rt_bool_t armed)
{
    rt_bool_t int_on;
    struct euler_float meas_angle, target_angle;
    meas_angle.roll = att_in->roll;
    meas_angle.pitch = att_in->pitch;
    meas_angle.yaw = att_in->yaw;
    target_angle.roll = (float)(rc_in->roll / 45.0f);
    target_angle.pitch = (float)(rc_in->pitch  / 45.0f);
    target_angle.yaw = (float)(rc_in->yaw / 45.0f);

    int_on = (armed && rc_in->throttle >= 10);

    /* 角度环 */
    pid_position_cal(&pid_roll_angle, target_angle.roll, meas_angle.roll, int_on);
    pid_position_cal(&pid_pitch_angle, target_angle.pitch, meas_angle.pitch, int_on);

    /* 角速度环 */
    pid_position_cal(&pid_roll_rate, pid_roll_angle.output,
                     gyros_in->y * RAD_TO_DEG, int_on);
    pid_position_cal(&pid_pitch_rate, pid_pitch_angle.output,
                     gyros_in->x * RAD_TO_DEG, int_on);
    pid_position_cal(&pid_yaw_rate, target_angle.yaw * pid_yaw_rate.p,
                     gyros_in->z * RAD_TO_DEG, int_on);

    if (int_on)
    {
        moto_pwm_1 = rc_in->throttle - pid_roll_rate.output + pid_pitch_angle.output
                     - pid_yaw_rate.output;
        moto_pwm_2 = rc_in->throttle + pid_roll_rate.output - pid_pitch_angle.output
                     + pid_yaw_rate.output;
        moto_pwm_3 = rc_in->throttle - pid_roll_rate.output + pid_pitch_angle.output
                     - pid_yaw_rate.output;
        moto_pwm_4 = rc_in->throttle + pid_roll_rate.output - pid_pitch_angle.output
                     + pid_yaw_rate.output;
    }
    else
    {
        moto_pwm_1 = 0;
        moto_pwm_2 = 0;
        moto_pwm_3 = 0;
        moto_pwm_4 = 0;
    }
    LOG_D("on %u pwm: %d %d %d %d", int_on, (int)moto_pwm_1, (int)moto_pwm_2,
          (int)moto_pwm_3, (int)moto_pwm_4);
    motor_set_speed(&motor1, moto_pwm_1);
    motor_set_speed(&motor2, moto_pwm_2);
    motor_set_speed(&motor3, moto_pwm_3);
    motor_set_speed(&motor4, moto_pwm_4);
}

static void control_recive_thread_entry(void *parameter)
{
    return;
}

static void check_airplane_status(void)
{
    if (g_imu_failed)
    {
        led_ctrl.led_mode = LED_MODE_IMU_FAIL;
    }
    else if (AIRPLANE_STATUS_LOCK == nb_control_status_get())
    {
        if (led_ctrl.led_mode != LED_MODE_FLIGHT_LOCK)
        {
            led_ctrl.led_mode = LED_MODE_FLIGHT_LOCK;
        }
    }
    else if (RT_FALSE == nb_remote_comm_ok())
    {
        if (led_ctrl.led_mode != LED_MODE_COMM_FAIL)
        {
            led_ctrl.led_mode = LED_MODE_COMM_FAIL;
        }
    }
    else
    {
        if (led_ctrl.led_mode != LED_MODE_FLIGHT_UNLOCK)
        {
            led_ctrl.led_mode = LED_MODE_FLIGHT_UNLOCK;
        }
    }

    if (led_ctrl.irq_sem != RT_NULL)
    {
        rt_sem_release(led_ctrl.irq_sem);
    }
}

static void control_thread_entry(void *parameter)
{
    rt_err_t result = 0;
    struct euler_float angles;
    struct vector3_float gyros;
    struct remote_control rc_in;
    rt_bool_t airplane_enable;

    unsigned long timestamp;

    if (motor_init(&motor1, "pwm3", 1, 500000, 4096) ||
        motor_init(&motor2, "pwm3", 2, 500000, 4096) ||
        motor_init(&motor3, "pwm3", 3, 500000, 4096) ||
        motor_init(&motor4, "pwm3", 4, 500000, 4096))
    {
        LOG_E("failed to initialize motors");
        return;
    }

    /* DMP 的中断频率设置为100Hz, 并以此频率驱动控制算法  */
    result = rt_mpu_init("i2c1", NB_MPU6050_IND_PIN, RT_TRUE, 100);
    if (result < 0)
    {
        LOG_E("rt_mpu6050_init failed");
        led_ctrl.led_mode = LED_MODE_RUN_ERROR;
        //rt_sem_release(led_ctrl.irq_sem);

        return;
    }

    LOG_D("rt_mpu6050_init OK");
    led_ctrl.led_mode = LED_MODE_FLIGHT_LOCK;
    //rt_sem_release(led_ctrl.irq_sem);

    while (1) {
        rt_get_uptime_ms(&timestamp);

        /* 正常情况下, mpu将以前面设定的频率产生中断, 这里也将以中断的频率获得信号量,
         * 设置一个超时时间, 确保发生异常的情况下不要一直阻塞 */
        if (!rt_sem_take(hal.irq_sem, rt_tick_from_millisecond(100)))
        {
            /* interrupt happened */
            rt_mpu_update(&angles, &gyros);
            g_imu_failed = RT_FALSE;
            // LOG_D("angles: %d %d %d, gyros: %d %d %d",
            //       (int)angles.pitch, (int)angles.roll, (int)angles.yaw,
            //       (int)gyros.x, (int)gyros.y, (int)gyros.z);
        }
        else
        {
            /* mpu 数据获取超时 */
            g_imu_failed = RT_TRUE;
            //rt_thread_delay(300);
            LOG_E("rt_mpu6050_update failed");
        }

        if (!g_imu_failed
            && nb_control_status_get() == LED_MODE_FLIGHT_UNLOCK
            && nb_remote_comm_ok() == RT_TRUE)
        {
            airplane_enable = RT_TRUE;
        }
        else
        {
            airplane_enable = RT_FALSE;
        }

        nb_remote_control_update(&rc_in);
        control(&angles, &gyros, &rc_in, airplane_enable);
        check_airplane_status();
    }
}

static int control_app_init(void)
{
    //rt_err_t err;
    rt_thread_t tid;

    tid = rt_thread_create("control_recv_thread",
                           control_recive_thread_entry, RT_NULL,
                           1024,
                           25,
                           5);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
    }
    else
    {
        LOG_E("init control_recv_thread failed");
        return -1;
    }

    tid = rt_thread_create("control_thread",
                           control_thread_entry, RT_NULL,
                           3072,  /* stack size */
                           22,    /* priority */
                           5);    /* time slice */
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
    }
    else
    {
        LOG_E("init control_thread failed");
        return -1;
    }

    return 0;
}

INIT_APP_EXPORT(control_app_init);
