/*******************************************************************************
 * @file     rtt_mpu_adpt.c
 * @author   lcc
 * @version
 * @date     2022-08-21
 * @brief
 ******************************************************************************/

#include <rtdevice.h>
#include <rtt_mpu_adpt.h>

static struct rt_i2c_bus_device *i2c_bus = RT_NULL;

int rt_mpu_i2c_init(const char *name)
{
    i2c_bus = (struct rt_i2c_bus_device *)rt_device_find(name);
    if (i2c_bus == RT_NULL)
    {
        rt_kprintf("can't find %s device!\n", name);
        return -1;
    }
    return 0;
}

int rt_mpu_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
                     unsigned short len, const unsigned char *data_ptr)
{
    rt_int8_t res = 0;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = slave_addr;     /* Slave address */
    msgs[0].flags = RT_I2C_WR;      /* Write flag */
    msgs[0].buf   = &reg_addr;      /* Slave register address */
    msgs[0].len   = 1;              /* Number of bytes sent */

    msgs[1].addr  = slave_addr;
    msgs[1].flags = RT_I2C_WR | RT_I2C_NO_START;
    msgs[1].buf   = (rt_uint8_t *)data_ptr;
    msgs[1].len   = len;

    if (rt_i2c_transfer(i2c_bus, msgs, 2) == 2)
    {
        res = RT_EOK;
    }
    else
    {
        res = -RT_ERROR;
    }

    return res;
}

int rt_mpu_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
                       unsigned short len, unsigned char *data_ptr)
{
    rt_int8_t res = 0;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = slave_addr;     /* Slave address */
    msgs[0].flags = RT_I2C_WR;      /* Write flag */
    msgs[0].buf   = &reg_addr;      /* Slave register address */
    msgs[0].len   = 1;              /* Number of bytes sent */

    msgs[1].addr  = slave_addr;     /* Slave address */
    msgs[1].flags = RT_I2C_RD;      /* Read flag */
    msgs[1].buf   = data_ptr;       /* Read data pointer */
    msgs[1].len   = len;            /* Number of bytes read */

    if (rt_i2c_transfer(i2c_bus, msgs, 2) == 2)
    {
        res = RT_EOK;
    }
    else
    {
        res = -RT_ERROR;
    }

    return res;
}

inline int rt_get_uptime_ms(unsigned long *ms)
{
    *ms = 1000 * rt_tick_get() / RT_TICK_PER_SECOND;
    return 0;
}
