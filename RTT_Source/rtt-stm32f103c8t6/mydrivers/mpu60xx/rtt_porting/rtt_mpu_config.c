/*******************************************************************************
 * @file     rtt_mpu_config.c
 * @author   lcc
 * @version
 * @date     2022-08-26
 * @brief
 ******************************************************************************/
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#include <rtthread.h>
#include <rtdevice.h>
#include "nb_common.h"
#include "rtt_mpu_adpt.h"
#include "rtt_mpu_config.h"
#include "board.h"
#ifdef __STM32F1XX_H
#include "stm32f1xx_hal_gpio_ex.h"
#endif

#include <math.h>

#define DBG_TAG     "mpu6050"
#define DBG_LVL     DBG_WARNING //3
#include <rtdbg.h>

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (200)

/* q30，q16 格式, long 转 float 时的除数. */
#define q30  1073741824.0f
#define q16  65536.0f

unsigned char *mpl_key = (unsigned char *)"eMPL 5.1";

/* Platform-specific information. Kinda like a boardfile. */
struct platform_data_s {
    signed char orientation[9];
};

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the
 * driver(s).
 * > 传感器可以以任何方向安装到板上。 下面看到的安装矩阵告诉 MPL 如何旋转来自驱
 * > 动程序的原始数据
 * TODO: The following matrices refer to the configuration on internal test
 * boards at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 * > TODO：以下矩阵参考 Invensense 内部测试板上的配置。 如果需要，请修改矩阵以匹配您的特定
 * > 设置的芯片到主体矩阵。
 */
static struct platform_data_s gyro_pdata = {
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};

struct hal_s hal = {0};

/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
void gyro_data_ready_cb(void *arg)
{
    hal.new_gyro = 1;
    rt_sem_release(hal.irq_sem);
    LOG_D("mpu release a semaphore");
}

rt_err_t rt_mpu_self_test(void)
{
    int result;
    //char test_packet[4] = {0};
    long gyro[3], accel[3];
    unsigned char i = 0;
#if defined (MPU6500) || defined (MPU9250)
    result = mpu_run_6500_self_test(gyro, accel, 0);
#elif defined (MPU6050) || defined (MPU9150)
    result = mpu_run_self_test(gyro, accel);
#endif
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        for(i = 0; i<3; i++) {
            gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
            accel[i] *= 2048.f; //convert to +-16G
            accel[i] = accel[i] >> 16;
            gyro[i] = (long)(gyro[i] >> 16);
        }

        mpu_set_gyro_bias_reg(gyro);

#if defined (MPU6500) || defined (MPU9250)
        mpu_set_accel_bias_6500_reg(accel);
#elif defined (MPU6050) || defined (MPU9150)
        mpu_set_accel_bias_6050_reg(accel);
#endif
        return 0;
    }
    else
        return -1;
}

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

rt_err_t rt_mpu_init(const char *bus_name, rt_base_t ind_pin,
                     rt_bool_t dmp_on, rt_uint16_t dmp_rate)
{
    rt_err_t result;
    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;
    unsigned short sample_rate;

    result = rt_mpu_i2c_init(bus_name);
    if (result) {
        LOG_E("Init I2C bus failed.\n");
        return -1;
    }

    result = mpu_init(NULL);
    if (result) {
        LOG_E("Could not initialize gyro.\n");
        return -1;
    }

    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
    result = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    rt_thread_mdelay(3);
    if (result) {
        LOG_E("Start sensors failed.\n");
        return -1;
    }

    /* Push both gyro and accel data into the FIFO. */
    result = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    if (result) {
        LOG_E("Set fifo failed.\n");
        return -1;
    }
    if (!dmp_on || dmp_rate < 100)
        sample_rate = DEFAULT_MPU_HZ;
    else {
        sample_rate = dmp_rate;
    }
    result = mpu_set_sample_rate(sample_rate);
    if (result) {
        LOG_E("set sample rate failed.\n");
        return -1;
    }

    /* Read back configuration in case it was set improperly. */
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);

    /* Initialize HAL state variables. */
    rt_memset(&hal, 0, sizeof(hal));
    hal.sensors = ACCEL_ON | GYRO_ON;

    if (!dmp_on)
        goto skip_dmp;

    /* To initialize the DMP:
     * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
     *    inv_mpu_dmp_motion_driver.h into the MPU memory.
     * 2. Push the gyro and accel orientation matrix to the DMP.
     * 3. Register gesture callbacks. Don't worry, these callbacks won't be
     *    executed unless the corresponding feature is enabled.
     * 4. Call dmp_enable_feature(mask) to enable different features.
     * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
     * 6. Call any feature-specific control functions.
     *
     * To enable the DMP, just call mpu_set_dmp_state(1). This function can
     * be called repeatedly to enable and disable the DMP at runtime.
     *
     * The following is a short summary of the features supported in the DMP
     * image provided in inv_mpu_dmp_motion_driver.c:
     * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
     * 200Hz. Integrating the gyro data at higher rates reduces numerical
     * errors (compared to integration on the MCU at a lower sampling rate).
     * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
     * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
     * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
     * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
     * an event at the four orientations where the screen should rotate.
     * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
     * no motion.
     * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
     * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
     * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
     * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
     */
    result = dmp_load_motion_driver_firmware();
    if (result) {
        LOG_E("Load firmware failed!");
        return -1;
    }
    result = dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
    if (result) {
        LOG_E("DMP set orientation failed!");
        return -1;
    }
    // dmp_register_tap_cb(tap_cb);
    // dmp_register_android_orient_cb(android_orient_cb);
    /*
     * Known Bug -
     * DMP when enabled will sample sensor data at 200Hz and output to FIFO at the rate
     * specified in the dmp_set_fifo_rate API. The DMP will then sent an interrupt once
     * a sample has been put into the FIFO. Therefore if the dmp_set_fifo_rate is at 25Hz
     * there will be a 25Hz interrupt from the MPU device.
     * > 采样速率 200Hz，然后使用 dmp_set_fifo_rate 设置的速率输出到 FIFO。每当一个采样数据被
     * > 输出到 FIFO，DMP 会产生一个中断。
     *
     * There is a known issue in which if you do not enable DMP_FEATURE_TAP
     * then the interrupts will be at 200Hz even if fifo rate
     * is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP
     * > 如果没使能 DMP_FEATURE_TAP, 中断将以 200Mhz 产生，而不管 dmp_set_fifo_rate 设置多少
     */
    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;
    result = dmp_enable_feature(hal.dmp_features);
    if (result) {
        LOG_E("DMP enable feature failed!");
        return -1;
    }
    result = dmp_set_fifo_rate(dmp_rate);
    if (result) {
        LOG_E("DMP set fifo rate failed!");
        return -1;
    }

    result = mpu_set_dmp_state(1);
    if (result) {
        LOG_E("Enable DMP failed!");
        return -1;
    }
    hal.dmp_on = 1;

    result = rt_mpu_self_test();
    if (result) {
        LOG_E("run_self_test failed!");
        return -1;
    }

skip_dmp:
    /* Attach a callback to the external interrupt */
    if (ind_pin >= 0) {
        LOG_D("creating semaphore mpu_irq");
        hal.irq_sem = rt_sem_create("mpu_irq", 0, RT_IPC_FLAG_FIFO);
        if (hal.irq_sem == NULL) {
            LOG_E("failed to alloc irq sem");
            return -1;
        }
#ifdef __STM32F1XX_H
        if (ind_pin == GET_PIN(B, 3)) {
            __HAL_RCC_AFIO_CLK_ENABLE();
            __HAL_AFIO_REMAP_SWJ_NOJTAG();
        }
#endif
        rt_pin_mode(ind_pin, PIN_MODE_INPUT);
        rt_pin_attach_irq(ind_pin, PIN_IRQ_MODE_RISING, gyro_data_ready_cb, RT_NULL);
        rt_pin_irq_enable(ind_pin, PIN_IRQ_ENABLE);
    }

    return 0;
}

/* @brief 得到 dmp 处理后的数据(注意, 本函数需要比较多堆栈, 局部变量有点多)
 * @param angles 姿态角  精度: 0.1° 范围: pitch -90.0° <---> +90.0°
 *                                     roll -180.0° <---> +180.0°
 *                                     yaw -180.0° <---> +180.0°
 * @param gyros 三轴角速度
 * @retval None.
 */
rt_err_t rt_mpu_update(struct euler_float *angles, struct vector3_float *gyros)
{
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    unsigned long sensor_timestamp;
    short gyro[3], accel[3], sensors;
    unsigned char more;
    long quat[4];

    if (hal.dmp_on) {
        if (dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more))
                return -1;
        if (sensors & INV_WXYZ_QUAT) {
            q0 = quat[0] / q30; //q30格式转换为浮点数
            q1 = quat[1] / q30;
            q2 = quat[2] / q30;
            q3 = quat[3] / q30;
            /* pitch 俯仰 */
            angles->pitch = asin(-2*q1*q3 + 2*q0*q2) * 57.3;
            /* roll 滚转 */
            angles->roll  = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1) * 57.3;
            /* yaw 偏航 */
            angles->yaw   = atan2(2*(q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3) * 57.3;
        }
        if (sensors & INV_XYZ_GYRO) {
            gyros->x = gyro[0];
            gyros->y = gyro[1];
            gyros->z = gyro[2];
        }
    }
    else {
        ;// mpu_read_fifo(), todo...
    }

    return 0;
}
