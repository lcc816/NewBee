/*******************************************************************************
 * @file     app_led.c
 * @author   lcc
 * @version  
 * @date     2022-07-09
 * @brief    
 ******************************************************************************/

#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>
#include <drv_spi.h>

#include "nb_common.h"
#include "rich_led.h"
#include <ws2812.h>
#include "app_common.h"

#define DBG_TAG "app.led_ctrl"
#define DBG_LVL NB_DBG_LVL
#include <rtdbg.h>

struct led_ctrl_s led_ctrl = {0};

static void nb_ws2812_spi_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    rt_hw_spi_device_attach(NB_WS2812B_SPI_BUS_NAME, NB_WS2812B_SPI_DEV_NAME,
                            NB_WS2812B_SPI_CS_PORT, NB_WS2812B_SPI_CS_PIN);
}

//static void nb_ws2812_test(ws2812_t ws2812)
//{
//#define COLOR_MIN_VALUE   10
//    ws2812_write_rgb_to_all(ws2812, 255, 0, 0);
//    ws2812_send(ws2812);
//    rt_thread_mdelay(2000);
//
//    ws2812_write_rgb_to_all(ws2812, 0, 200, 0);
//    ws2812_send(ws2812);
//    rt_thread_mdelay(2000);
//
//    ws2812_write_rgb_to_all(ws2812, 0, 0, 200);
//    ws2812_send(ws2812);
//    rt_thread_mdelay(2000);
//
//    ws2812_write_rgb_to_all(ws2812, 0, 200, 200);
//    ws2812_send(ws2812);
//    rt_thread_mdelay(2000);
//
//    ws2812_write_rgb_to_all(ws2812, 200, 0, 200);
//    ws2812_send(ws2812);
//    rt_thread_mdelay(2000);
//
//    ws2812_write_rgb_to_all(ws2812, 200, 200, 0);
//    ws2812_send(ws2812);
//    rt_thread_mdelay(2000);
//
//    ws2812_write_rgb_to_all(ws2812, 1, 1, 1);
//    ws2812_send(ws2812);
//}

void nb_led_ticks_timeout(void *parameter)
{
    rich_led_action_ticks();
}

static void led_mode_set(void)
{
    switch (led_ctrl.led_mode)
    {
    case LED_MODE_FLIGHT_LOCK:
        rich_led_action_set(ACTION_ID_AIRPLANE_LOCK);
        break;
    case LED_MODE_FLIGHT_UNLOCK:
        rich_led_action_set(ACTION_ID_AIRPLANE_UNLOCK);
        break;
    case LED_MODE_STARTING:
        rich_led_action_set(ACTION_ID_SYSTEM_STARTING);
        break;
    case LED_MODE_RUN_ERROR:
        rich_led_action_set(ACTION_ID_SYSTEM_ERROR);
        break;
    case LED_MODE_COMM_FAIL:
        rich_led_action_set(ACTION_ID_AIRPLANE_DISCONNECTED);
        break;
    default:
        break;
    }
}

void nb_led_ctrl_thread_entry(void *parameter)
{
    rich_led_instance_init();
    led_mode_set();
    nb_ws2812_spi_init();
    ws2812_t ws2812 = ws2812_create(NB_WS2812B_SPI_DEV_NAME,
                                    NB_WS2812B_NODE_LENGTH);
    if (!ws2812)
    {
        LOG_E("create ws2812 object faild.\r\n");
        return;
    }

    LOG_E("led ctrl init ok");

    //nb_ws2812_test(ws2812);

    while (1)
    {
        if (!rt_sem_take(led_ctrl.irq_sem, rt_tick_from_millisecond(100)))
        {
            LOG_D("led changed to %u", led_ctrl.led_mode);
        }
        led_mode_set();
        // to do other things
    }
}

static int led_app_init(void)
{
    rt_thread_t tid;
    rt_timer_t timer;

    led_ctrl.led_mode = LED_MODE_FLIGHT_LOCK;
    led_ctrl.irq_sem = rt_sem_create("led_irq", 0, RT_IPC_FLAG_FIFO);
    if (led_ctrl.irq_sem == NULL) {
        LOG_E("failed to alloc irq sem");
        return -1;
    }

    timer = rt_timer_create("led_ticks_timer", nb_led_ticks_timeout,
                            RT_NULL, 1,
                            RT_TIMER_FLAG_PERIODIC);

    if (timer != RT_NULL)
    {
        rt_timer_start(timer);
    }
    else
    {
        return -1;
    }

    tid = rt_thread_create("led_ctrl_thread", nb_led_ctrl_thread_entry, RT_NULL,
                           1024,    /* stack size */
                           28,      /* priority */
                           5);      /* time slice */
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
    }
    else
    {
        return -1;
    }
    return 0;
}

INIT_DEVICE_EXPORT(led_app_init);
