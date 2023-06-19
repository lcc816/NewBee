/*******************************************************************************
 * @file     rtt_mpu_adpt.h
 * @author   lcc
 * @version
 * @date     2022-08-20
 * @brief
 ******************************************************************************/

#ifndef __RTT_ADPT_H_
#define __RTT_ADPT_H_

#include <rtthread.h>

int rt_mpu_i2c_init(const char *name);

int rt_mpu_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
                        unsigned short len, const unsigned char *data_ptr);

int rt_mpu_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
                       unsigned short len, unsigned char *data_ptr);

int rt_get_uptime_ms(unsigned long *ms);

#endif /* __RTT_ADPT_H_ */
