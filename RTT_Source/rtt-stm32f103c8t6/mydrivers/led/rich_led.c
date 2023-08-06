/*******************************************************************************
 * @file     rich_led.c
 * @author   lcc
 * @version
 * @date     2023-04-23
 * @brief
 ******************************************************************************/

#include "rich_led.h"
#include <string.h>

/* Uncomment the line below to debug and assert */
//#define RICH_LED_DEBUG

#ifdef RICH_LED_DEBUG
#include <rtthread.h>
#define DEBUG_PRINT(arg, ...) rt_kprintf(arg, ##__VA_ARGS__)
#define LED_ASSERT(expr) ((expr) ? (void)0 : \
            (void)rt_kprintf("assert failed at %s %d\r\n", __FILE__, __LINE__))
#else
#define DEBUG_PRINT(arg, ...)
#define LED_ASSERT(expr) ((void)0)
#endif

extern struct rich_led_action action_instance[];
extern struct rich_led_config led_instance[];
extern struct rich_led_record action_record[NUM_LED_ID];

void rich_led_instance_init(void)
{
    int i;
    struct rich_led_action *act;
    for (i = 0; i <= LED_ID_MAX; i++)
    {
        led_instance[i].hal_gpio_init();
        led_instance[i].hal_led_off();
    }
    memset(action_record, 0, sizeof(action_record));
    // Check whether the action parameters are valid
    for (i = 0; i <= ACTION_ID_MAX; i++)
    {
        act = &action_instance[i];
        LED_ASSERT(act->led_id <= LED_ID_MAX);
        if (act->duration > 0)
        {
            LED_ASSERT(act->duration >= (act->pulse_low + act->pulse_high) * act->pulse_num_per_cycle);
        }
    }
}

void rich_led_action_set(uint32_t action_id)
{
    uint32_t led_id;

    if (action_id == 0 || action_id > ACTION_ID_MAX)
    {
        // invalid action id
        return;
    }

    led_id = action_instance[action_id].led_id;
    if (action_record[led_id].action_id != action_id)
    {
        action_record[led_id].action_id = action_id;
        action_record[led_id].action_changed = 1;
        DEBUG_PRINT("set to action %u!\r\n", action_id);
    }
}

void led_action_turn_off_all(void)
{
    uint32_t led_id;

    for (led_id = 0; led_id <= LED_ID_MAX; led_id++)
    {
        action_record[led_id].action_id = ACITON_ID_NONE;
        led_instance[led_id].hal_led_off();
    }
}

void rich_led_action_ticks(void)
{
    int i;
    struct rich_led_action *action;
    struct rich_led_record *record;
    uint16_t pulse;
    bool state_changed;
    for (i = 0; i <= LED_ID_MAX; i++)
    {
        record = &action_record[i];
        if (record->action_changed)
        {
            record->action_changed = 0;
            record->pulse_cnt = 0;
            record->ticks = 0;
            led_instance[i].hal_led_off();
            DEBUG_PRINT("led %d switch action\r\n", i);
        }
        if (record->action_id == ACITON_ID_NONE)
            continue;

        action = &action_instance[record->action_id];
        // Normally on
        if (action->pulse_low == 0)
        {
            if (record->current_state != LED_STATE_ON)
            {
                record->current_state = LED_STATE_ON;
                led_instance[i].hal_led_on();
            }
            continue;
        }
        // Normally off
        if (action->pulse_high == 0)
        {
            if (record->current_state != LED_STATE_OFF)
            {
                record->current_state = LED_STATE_OFF;
                led_instance[i].hal_led_off();
            }
            continue;
        }

        state_changed = false;
        if (record->ticks == 0)
        {
            record->pulse_cnt = 1;
            DEBUG_PRINT("led id %d, pulse cnt%d\r\n", i, record->pulse_cnt);
        }
        record->ticks++;

        pulse = (action->pulse_low + action->pulse_high);

        if (record->pulse_cnt <= action->pulse_num_per_cycle)
        {
            if (record->ticks % pulse < action->pulse_low)
            {
                if (record->current_state != LED_STATE_OFF)
                {
                    record->current_state = LED_STATE_OFF;
                    state_changed = true;
                }
            }
            else
            {
                if (record->current_state != LED_STATE_ON)
                {
                    record->current_state = LED_STATE_ON;
                    state_changed = true;
                }
            }
        }

        if (state_changed)
        {
            if (record->current_state == LED_STATE_OFF)
            {
                led_instance[i].hal_led_off();
                DEBUG_PRINT("led id %d, low\r\n", i);
                record->pulse_cnt++;
                DEBUG_PRINT("led id %d, pulse cnt%d\r\n", i, record->pulse_cnt);
            }
            else
            {
                led_instance[i].hal_led_on();
                DEBUG_PRINT("led id %d, high\r\n", i);
            }
        }

        if (record->pulse_cnt == action->pulse_num_per_cycle)
        {
            if (action->duration == -1)
            {
                record->pulse_cnt = 0;
                // no repeat, set action to none
                record->action_id = ACITON_ID_NONE;
                record->action_changed = 1;
            }
        }
        if (record->ticks == action->duration)
        {
            record->pulse_cnt = 0;
            record->ticks = 0;
        }
    }
}
