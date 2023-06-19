/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-30     lcc          the first version
 */

#ifndef __NRF24L01_STM32_ADAPTER_H_
#define __NRF24L01_STM32_ADAPTER_H_

/* STM32 GPIO driver */
struct stm32_pin_index
{
    int index;
    GPIO_TypeDef *gpio;
    uint32_t pin;
};

void nrf24l01_spi_csn_config(rt_base_t csn_pin);

const struct stm32_pin_index *get_stm32_pin(uint8_t pin);

#endif /* __NRF24L01_STM32_ADAPTER_H_ */
