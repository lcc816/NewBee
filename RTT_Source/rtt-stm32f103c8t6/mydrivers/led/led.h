/*******************************************************************************
 * @file     led.h
 * @author   lcc
 * @version  
 * @date     2022-05-15
 * @brief    
 ******************************************************************************/

#ifndef MYDRIVERS_LED_LED_H_
#define MYDRIVERS_LED_LED_H_

#include "nb_common.h"

void nb_led_init(void);
void nb_led_off(void);
void nb_led_on(void);
void nb_led_flip(void);

#endif /* MYDRIVERS_LED_LED_H_ */
