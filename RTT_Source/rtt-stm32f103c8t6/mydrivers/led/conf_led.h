/*******************************************************************************
 * @file     conf_led.h
 * @author   lcc
 * @version
 * @date     2023-05-10
 * @brief
 ******************************************************************************/

enum _led_id
{
    LED_ID_GREEN = 0,
    //LED_ID_BLUE,
    NUM_LED_ID,
    LED_ID_MAX = NUM_LED_ID - 1
};

enum _action_id
{
    ACITON_ID_NONE = 0,
    ACTION_ID_SYSTEM_ERROR,
    ACTION_ID_AIRPLANE_DISCONNECTED,
    ACTION_ID_AIRPLANE_LOCK,
    ACTION_ID_AIRPLANE_UNLOCK,
    ACTION_ID_BATTERY_LOW,
    ACTION_ID_IMU_ERROR,
    NUM_ACTION_ID,
    ACTION_ID_MAX = NUM_ACTION_ID - 1
};
