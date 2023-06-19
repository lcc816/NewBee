/*******************************************************************************
 * @file     rtt_mpu_config.h
 * @author   lcc
 * @version
 * @date     2022-08-26
 * @brief
 ******************************************************************************/
#ifndef __RTT_MPU_CONFIG_H_
#define __RTT_MPU_CONFIG_H_

//#define PEDO_READ_MS    (1000)
//#define TEMP_READ_MS    (500)
//#define COMPASS_READ_MS (100)

struct rx_s
{
    unsigned char header[3];
    unsigned char cmd;
};

struct hal_s {
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    volatile rt_sem_t irq_sem;
    unsigned char motion_int_mode;
    unsigned long next_temp_ms;
    unsigned short dmp_features;
    struct rx_s rx;
};

struct vector3_float
{
    float x;
    float y;
    float z;
};

struct euler_int
{
    rt_int16_t pitch;
    rt_int16_t roll;
    rt_int16_t yaw;
};

struct euler_float
{
    float pitch;
    float roll;
    float yaw;
};

extern struct hal_s hal;

rt_err_t rt_mpu_init(const char *bus_name, rt_base_t ind_pin,
                     rt_bool_t dmp_on, rt_uint16_t dmp_rate);
rt_err_t rt_mpu_self_test(void);
rt_err_t rt_mpu_update(struct euler_float *angles, struct vector3_float *gyros);

#endif /* __RTT_MPU_CONFIG_H_ */
