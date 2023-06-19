/*******************************************************************************
 * @file     main.c
 * @author   lcc
 * @version  
 * @date     2022-07-10
 * @brief    
 ******************************************************************************/

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "nb_common.h"

#define DBG_TAG "main"
#define DBG_LVL GLOBAL_DBG_LVL//DBG_LOG
#include <rtdbg.h>

int main(void)
{
    int count = 1;

    while (1)
    {
        count++;
        //LOG_D("Hello RT-Thread! %d", count);
        rt_thread_mdelay(500);
    }

    return RT_EOK;
}
