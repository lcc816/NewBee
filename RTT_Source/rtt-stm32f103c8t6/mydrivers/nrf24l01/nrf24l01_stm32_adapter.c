/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-30     Lichangchun       the first version
 */

#include "drv_common.h"
#include "nrf24l01_stm32_adapter.h"

#define __STM32_PIN(index, gpio, gpio_index)                                \
    {                                                                       \
        index, GPIO##gpio, GPIO_PIN_##gpio_index                            \
    }

static const struct stm32_pin_index pins[] =
{
#if defined(GPIOA)
    __STM32_PIN(0 ,  A, 0 ),
    __STM32_PIN(1 ,  A, 1 ),
    __STM32_PIN(2 ,  A, 2 ),
    __STM32_PIN(3 ,  A, 3 ),
    __STM32_PIN(4 ,  A, 4 ),
    __STM32_PIN(5 ,  A, 5 ),
    __STM32_PIN(6 ,  A, 6 ),
    __STM32_PIN(7 ,  A, 7 ),
    __STM32_PIN(8 ,  A, 8 ),
    __STM32_PIN(9 ,  A, 9 ),
    __STM32_PIN(10,  A, 10),
    __STM32_PIN(11,  A, 11),
    __STM32_PIN(12,  A, 12),
    __STM32_PIN(13,  A, 13),
    __STM32_PIN(14,  A, 14),
    __STM32_PIN(15,  A, 15),
#if defined(GPIOB)
    __STM32_PIN(16,  B, 0),
    __STM32_PIN(17,  B, 1),
    __STM32_PIN(18,  B, 2),
    __STM32_PIN(19,  B, 3),
    __STM32_PIN(20,  B, 4),
    __STM32_PIN(21,  B, 5),
    __STM32_PIN(22,  B, 6),
    __STM32_PIN(23,  B, 7),
    __STM32_PIN(24,  B, 8),
    __STM32_PIN(25,  B, 9),
    __STM32_PIN(26,  B, 10),
    __STM32_PIN(27,  B, 11),
    __STM32_PIN(28,  B, 12),
    __STM32_PIN(29,  B, 13),
    __STM32_PIN(30,  B, 14),
    __STM32_PIN(31,  B, 15),
#if defined(GPIOC)
    __STM32_PIN(32,  C, 0),
    __STM32_PIN(33,  C, 1),
    __STM32_PIN(34,  C, 2),
    __STM32_PIN(35,  C, 3),
    __STM32_PIN(36,  C, 4),
    __STM32_PIN(37,  C, 5),
    __STM32_PIN(38,  C, 6),
    __STM32_PIN(39,  C, 7),
    __STM32_PIN(40,  C, 8),
    __STM32_PIN(41,  C, 9),
    __STM32_PIN(42,  C, 10),
    __STM32_PIN(43,  C, 11),
    __STM32_PIN(44,  C, 12),
    __STM32_PIN(45,  C, 13),
    __STM32_PIN(46,  C, 14),
    __STM32_PIN(47,  C, 15),
#if defined(GPIOD)
    __STM32_PIN(48,  D, 0),
    __STM32_PIN(49,  D, 1),
    __STM32_PIN(50,  D, 2),
    __STM32_PIN(51,  D, 3),
    __STM32_PIN(52,  D, 4),
    __STM32_PIN(53,  D, 5),
    __STM32_PIN(54,  D, 6),
    __STM32_PIN(55,  D, 7),
    __STM32_PIN(56,  D, 8),
    __STM32_PIN(57,  D, 9),
    __STM32_PIN(58,  D, 10),
    __STM32_PIN(59,  D, 11),
    __STM32_PIN(60,  D, 12),
    __STM32_PIN(61,  D, 13),
    __STM32_PIN(62,  D, 14),
    __STM32_PIN(63,  D, 15),
#if defined(GPIOE)
    __STM32_PIN(64,  E, 0),
    __STM32_PIN(65,  E, 1),
    __STM32_PIN(66,  E, 2),
    __STM32_PIN(67,  E, 3),
    __STM32_PIN(68,  E, 4),
    __STM32_PIN(69,  E, 5),
    __STM32_PIN(70,  E, 6),
    __STM32_PIN(71,  E, 7),
    __STM32_PIN(72,  E, 8),
    __STM32_PIN(73,  E, 9),
    __STM32_PIN(74,  E, 10),
    __STM32_PIN(75,  E, 11),
    __STM32_PIN(76,  E, 12),
    __STM32_PIN(77,  E, 13),
    __STM32_PIN(78,  E, 14),
    __STM32_PIN(79,  E, 15),
#if defined(GPIOF)
    __STM32_PIN(80,  F, 0),
    __STM32_PIN(81,  F, 1),
    __STM32_PIN(82,  F, 2),
    __STM32_PIN(83,  F, 3),
    __STM32_PIN(84,  F, 4),
    __STM32_PIN(85,  F, 5),
    __STM32_PIN(86,  F, 6),
    __STM32_PIN(87,  F, 7),
    __STM32_PIN(88,  F, 8),
    __STM32_PIN(89,  F, 9),
    __STM32_PIN(90,  F, 10),
    __STM32_PIN(91,  F, 11),
    __STM32_PIN(92,  F, 12),
    __STM32_PIN(93,  F, 13),
    __STM32_PIN(94,  F, 14),
    __STM32_PIN(95,  F, 15),
#if defined(GPIOG)
    __STM32_PIN(96,  G, 0),
    __STM32_PIN(97,  G, 1),
    __STM32_PIN(98,  G, 2),
    __STM32_PIN(99,  G, 3),
    __STM32_PIN(100, G, 4),
    __STM32_PIN(101, G, 5),
    __STM32_PIN(102, G, 6),
    __STM32_PIN(103, G, 7),
    __STM32_PIN(104, G, 8),
    __STM32_PIN(105, G, 9),
    __STM32_PIN(106, G, 10),
    __STM32_PIN(107, G, 11),
    __STM32_PIN(108, G, 12),
    __STM32_PIN(109, G, 13),
    __STM32_PIN(110, G, 14),
    __STM32_PIN(111, G, 15),
#if defined(GPIOH)
    __STM32_PIN(112, H, 0),
    __STM32_PIN(113, H, 1),
    __STM32_PIN(114, H, 2),
    __STM32_PIN(115, H, 3),
    __STM32_PIN(116, H, 4),
    __STM32_PIN(117, H, 5),
    __STM32_PIN(118, H, 6),
    __STM32_PIN(119, H, 7),
    __STM32_PIN(120, H, 8),
    __STM32_PIN(121, H, 9),
    __STM32_PIN(122, H, 10),
    __STM32_PIN(123, H, 11),
    __STM32_PIN(124, H, 12),
    __STM32_PIN(125, H, 13),
    __STM32_PIN(126, H, 14),
    __STM32_PIN(127, H, 15),
#if defined(GPIOI)
    __STM32_PIN(128, I, 0),
    __STM32_PIN(129, I, 1),
    __STM32_PIN(130, I, 2),
    __STM32_PIN(131, I, 3),
    __STM32_PIN(132, I, 4),
    __STM32_PIN(133, I, 5),
    __STM32_PIN(134, I, 6),
    __STM32_PIN(135, I, 7),
    __STM32_PIN(136, I, 8),
    __STM32_PIN(137, I, 9),
    __STM32_PIN(138, I, 10),
    __STM32_PIN(139, I, 11),
    __STM32_PIN(140, I, 12),
    __STM32_PIN(141, I, 13),
    __STM32_PIN(142, I, 14),
    __STM32_PIN(143, I, 15),
#if defined(GPIOJ)
    __STM32_PIN(144, J, 0),
    __STM32_PIN(145, J, 1),
    __STM32_PIN(146, J, 2),
    __STM32_PIN(147, J, 3),
    __STM32_PIN(148, J, 4),
    __STM32_PIN(149, J, 5),
    __STM32_PIN(150, J, 6),
    __STM32_PIN(151, J, 7),
    __STM32_PIN(152, J, 8),
    __STM32_PIN(153, J, 9),
    __STM32_PIN(154, J, 10),
    __STM32_PIN(155, J, 11),
    __STM32_PIN(156, J, 12),
    __STM32_PIN(157, J, 13),
    __STM32_PIN(158, J, 14),
    __STM32_PIN(159, J, 15),
#if defined(GPIOK)
    __STM32_PIN(160, K, 0),
    __STM32_PIN(161, K, 1),
    __STM32_PIN(162, K, 2),
    __STM32_PIN(163, K, 3),
    __STM32_PIN(164, K, 4),
    __STM32_PIN(165, K, 5),
    __STM32_PIN(166, K, 6),
    __STM32_PIN(167, K, 7),
    __STM32_PIN(168, K, 8),
    __STM32_PIN(169, K, 9),
    __STM32_PIN(170, K, 10),
    __STM32_PIN(171, K, 11),
    __STM32_PIN(172, K, 12),
    __STM32_PIN(173, K, 13),
    __STM32_PIN(174, K, 14),
    __STM32_PIN(175, K, 15),
#endif /* defined(GPIOK) */
#endif /* defined(GPIOJ) */
#endif /* defined(GPIOI) */
#endif /* defined(GPIOH) */
#endif /* defined(GPIOG) */
#endif /* defined(GPIOF) */
#endif /* defined(GPIOE) */
#endif /* defined(GPIOD) */
#endif /* defined(GPIOC) */
#endif /* defined(GPIOB) */
#endif /* defined(GPIOA) */
};

#define ITEM_NUM(items) sizeof(items) / sizeof(items[0])
const struct stm32_pin_index *get_stm32_pin(uint8_t pin)
{
    const struct stm32_pin_index *index;

    if (pin < ITEM_NUM(pins))
    {
        index = &pins[pin];
        if (index->index == -1)
            index = RT_NULL;
    }
    else
    {
        index = RT_NULL;
    }

    return index;
};

void nrf24l01_spi_csn_config(rt_base_t csn_pin)
{
    const struct stm32_pin_index *index;
    GPIO_InitTypeDef GPIO_InitStruct;

    index = get_stm32_pin(csn_pin);
    if (index == RT_NULL)
    {
        return;
    }

    GPIO_InitStruct.Pin = index->pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(index->gpio, &GPIO_InitStruct);
    HAL_GPIO_WritePin(index->gpio, index->pin, GPIO_PIN_SET);
}
