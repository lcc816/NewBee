/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-05-15     Lichangchun       the first version
 */

#include <rtdevice.h>
#include <board.h>
#include <led.h>

void nb_led_init(void)
{
    rt_pin_mode(NB_LED0_PIN, PIN_MODE_OUTPUT);
}

void nb_led_off(void)
{
    rt_pin_write(NB_LED0_PIN, PIN_HIGH);
}

void nb_led_on(void)
{
    rt_pin_write(NB_LED0_PIN, PIN_LOW);
}

void nb_led_flip(void)
{
    if (rt_pin_read(NB_LED0_PIN))
    {
        rt_pin_write(NB_LED0_PIN, PIN_LOW);
    }
    else
    {
        rt_pin_write(NB_LED0_PIN, PIN_HIGH);
    }
}
