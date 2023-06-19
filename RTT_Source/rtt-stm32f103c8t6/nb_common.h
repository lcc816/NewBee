/*******************************************************************************
 * @file     nb_common.h
 * @author   lcc
 * @version  
 * @date     2022-07-06
 * @brief    
 ******************************************************************************/

#ifndef __NB_COMMON_H_
#define __NB_COMMON_H_

#define GLOBAL_DBG_LVL      5 // modfied this when building release

/*---------------------------- LED CONFIG -----------------------------*/
#define NB_LED0_PIN         GET_PIN(A, 4)
//#define NB_LED1_PIN         GET_PIN(D, 2)

/*-------------------------- NRF24L01 CONFIG --------------------------*/
#define RF24_TX_PAYLOAD_WIDTH   32
#define RF24_RX_PAYLOAD_WIDTH   32

/*--------------------------- WS2812B CONFIG --------------------------*/
#define NB_WS2812B_SPI_BUS_NAME "spi2"
#define NB_WS2812B_SPI_DEV_NAME "spi20"
#define NB_WS2812B_SPI_CS_PORT  GPIOB
#define NB_WS2812B_SPI_CS_PIN   GPIO_PIN_12
#define NB_WS2812B_NODE_LENGTH  8

/*-------------------------- MPU6050 CONFIG ---------------------------*/
#define NB_MPU6050_I2C_BUS_NAME     "i2c1"
#define NB_MPU6050_I2C_ADDR         0x68
#define NB_MPU6050_IND_PIN          GET_PIN(B, 3)
#define MPU6050
#define EMPL_TARGET_RTT
#define EMPL
#define MPL_LOG_LVL                 MPL_LOG_DEBUG

#endif /* __NB_COMMON_H_ */
