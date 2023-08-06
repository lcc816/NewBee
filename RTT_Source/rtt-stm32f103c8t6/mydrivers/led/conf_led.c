/*******************************************************************************
 * @file     conf_led.c
 * @author   lcc
 * @version
 * @date     2023-05-10
 * @brief
 ******************************************************************************/

#include "rich_led.h"
#include "led.h"

struct rich_led_config led_instance[NUM_LED_ID] =
{
    [LED_ID_GREEN] = {nb_led_init, nb_led_on, nb_led_off},
    //[LED_ID_BLUE] = {led_blue_gpio_init, led_blue_on, led_blue_off}
};

struct rich_led_action action_instance[] =
{
    [ACTION_ID_SYSTEM_ERROR] = {LED_ID_GREEN, 500, 250, 250, 1},
    [ACTION_ID_AIRPLANE_DISCONNECTED] = {LED_ID_GREEN, 1000, 100, 100, 3},
    [ACTION_ID_AIRPLANE_UNLOCK] = {LED_ID_GREEN, -1, 0, 1, 1},
    [ACTION_ID_AIRPLANE_LOCK] = {LED_ID_GREEN, 1000, 500, 500, 1},
    [ACTION_ID_BATTERY_LOW] = {LED_ID_GREEN, 3000, 500, 500, 1},
    [ACTION_ID_IMU_ERROR] = {LED_ID_GREEN, 1000, 150, 150, 2},
};

struct rich_led_record action_record[NUM_LED_ID];
