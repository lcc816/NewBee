/*******************************************************************************
 * @file     rich_led.h
 * @author   lcc
 * @version
 * @date     2023-04-23
 * @brief
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "conf_led.h"

struct led_config
{
    void (*hal_gpio_init)(void);
    void (*hal_led_on)(void);
    void (*hal_led_off)(void);
};

struct led_action
{
    uint32_t led_id;
    int16_t  duration;
    int16_t  pulse_low;
    int16_t  pulse_high;
    uint8_t  pulse_num_per_cycle;
};

struct led_action_record
{
    uint32_t action_id;
    uint16_t ticks;
    uint8_t pulse_cnt;
    uint8_t current_state;
    uint8_t action_changed;
};

enum led_state
{
    LED_STATE_OFF = 0,
    LED_STATE_ON = 1
};

void led_instance_init(void);

void led_action_set(uint32_t action_id);

void action_ticks(void);
