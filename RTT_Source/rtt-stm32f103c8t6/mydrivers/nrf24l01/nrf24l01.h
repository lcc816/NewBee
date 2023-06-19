/*******************************************************************************
 * @file     nrf24l01.h
 * @author   lcc
 * @version  
 * @date     2022-02-14
 * @brief    
 ******************************************************************************/

#ifndef __NRF24L01_H_
#define __NRF24L01_H_

#include <rtthread.h>

/* Memory Map */
#define RF24_REG_CONFIG      0x00
#define RF24_REG_EN_AA       0x01
#define RF24_REG_EN_RXADDR   0x02
#define RF24_REG_SETUP_AW    0x03
#define RF24_REG_SETUP_RETR  0x04
#define RF24_REG_RF_CH       0x05
#define RF24_REG_RF_SETUP    0x06
#define RF24_REG_STATUS      0x07
#define RF24_REG_OBSERVE_TX  0x08
#define RF24_REG_RPD         0x09
#define RF24_REG_RX_ADDR_P0  0x0A
#define RF24_REG_RX_ADDR_P1  0x0B
#define RF24_REG_RX_ADDR_P2  0x0C
#define RF24_REG_RX_ADDR_P3  0x0D
#define RF24_REG_RX_ADDR_P4  0x0E
#define RF24_REG_RX_ADDR_P5  0x0F
#define RF24_REG_TX_ADDR     0x10
#define RF24_REG_RX_PW_P0    0x11
#define RF24_REG_RX_PW_P1    0x12
#define RF24_REG_RX_PW_P2    0x13
#define RF24_REG_RX_PW_P3    0x14
#define RF24_REG_RX_PW_P4    0x15
#define RF24_REG_RX_PW_P5    0x16
#define RF24_REG_FIFO_STATUS 0x17
#define RF24_REG_DYNPD       0x1C
#define RF24_REG_FEATRUE     0x1D

/* Bit Mnemonics */
/* 00 - CONFIG */
#define RF24_SHIFT_MASK_RX_DR  6
#define RF24_SHIFT_MASK_TX_DS  5
#define RF24_SHIFT_MASK_MAX_RT 4
#define RF24_SHIFT_EN_CRC      3
#define RF24_SHIFT_CRCO        2
#define RF24_SHIFT_PWR_UP      1
#define RF24_SHIFT_PRIM_RX     0
/* 01 - EN_AA */
#define RF24_SHIFT_ENAA_P5     5
#define RF24_SHIFT_ENAA_P4     4
#define RF24_SHIFT_ENAA_P3     3
#define RF24_SHIFT_ENAA_P2     2
#define RF24_SHIFT_ENAA_P1     1
#define RF24_SHIFT_ENAA_P0     0
/* 02 - EN_RXADDR */
#define RF24_SHIFT_ERX_P5      5
#define RF24_SHIFT_ERX_P4      4
#define RF24_SHIFT_ERX_P3      3
#define RF24_SHIFT_ERX_P2      2
#define RF24_SHIFT_ERX_P1      1
#define RF24_SHIFT_ERX_P0      0
/* 03 - SETUP_AW */
#define RF24_SHIFT_AW          0
/* 04 - SETUP_RETR */
#define RF24_SHIFT_ARD         4
#define RF24_SHIFT_ARC         0
/* 05 - RF_CH */
#define RF24_SHIFT_PLL_LOCK    4
/* 06 - RF_SETUP */
#define RF24_SHIFT_CONT_WAVE   7
#define RF24_SHIFT_RF_DR_LOW   5
#define RF24_SHIFT_RF_DR_HIGH  3
#define RF24_SHIFT_RF_PWR      1
/* 07 - STATUS */
#define RF24_SHIFT_RX_DR       6
#define RF24_SHIFT_TX_DS       5
#define RF24_SHIFT_MAX_RT      4
#define RF24_SHIFT_RX_P_NO     1
#define RF24_SHIFT_TX_FULL     0
/* 08 - OBSERVE_TX */
#define RF24_SHIFT_PLOS_CNT    4
#define RF24_SHIFT_ARC_CNT     0
/* 17 - FIFO_STATUS */
#define RF24_SHIFT_TX_REUSE    6
#define RF24_SHIFT_FIFO_FULL   5
#define RF24_SHIFT_TX_EMPTY    4
#define RF24_SHIFT_RX_FULL     1
#define RF24_SHIFT_RX_EMPTY    0
/* 1C - DYNPD */
#define RF24_SHIFT_DPL_P5      5
#define RF24_SHIFT_DPL_P4      4
#define RF24_SHIFT_DPL_P3      3
#define RF24_SHIFT_DPL_P2      2
#define RF24_SHIFT_DPL_P1      1
#define RF24_SHIFT_DPL_P0      0
/* 1D - FEATURE */
#define RF24_SHIFT_EN_DPL      2
#define RF24_SHIFT_EN_ACK_PAY  1
#define RF24_SHIFT_EN_DYN_ACK  0

/* Instruction Mnemonics */
#define RF24_CMD_R_REGISTER    0x00
#define RF24_CMD_W_REGISTER    0x20
#define RF24_CMD_REGISTER_MASK 0x1F
#define RF24_CMD_R_RX_PAYLOAD  0x61
#define RF24_CMD_W_TX_PAYLOAD  0xA0
#define RF24_CMD_FLUSH_TX      0xE1
#define RF24_CMD_FLUSH_RX      0xE2
#define RF24_CMD_REUSE_TX_PL   0xE3
#define RF24_CMD_R_RX_PL_WID   0x60
#define RF24_CMD_W_ACK_PAYLOAD 0xA0
#define RF24_CMD_W_TX_PAYLOAD_NOACK  0xB0
#define RF24_CMD_NOP           0xFF

/* Status */
#define STATUS_OK       0
#define STATUS_FAIL     -1
#define STATUS_MAX_RT   -2
#define STATUS_TIMEOUT  -3

typedef enum {
    RF24_MODE_POWROFF,
    RF24_MODE_UNCONF,
    RF24_MODE_TX,
    RF24_MODE_RX
} rf24_mode_e;

typedef enum {
    /** (0) represents 1 Mbps */
    RF24_1MBPS = 0,
    /** (1) represents 2 Mbps */
    RF24_2MBPS,
    /** (2) represents 250 kbps */
    RF24_250KBPS
} rf24_datarate_e;

/*
 * | level (enum value) | nRF24L01 | Si24R1 ext = 1 | Si24R1 ext = 0 |
 * |:------------------:|:-------:|:--------:|:-------:|
 * |   RF24_PWR_LVL0    | -18 dBm |  -6 dBm  | -12 dBm |
 * |   RF24_PWR_LVL1    | -12 dBm |  -0 dBm  | -4 dBm  |
 * |   RF24_PWR_LVL2    | -6 dBm  |  3 dBm   | 1 dBm   |
 * |   RF24_PWR_LVL3    |  0 dBm  |  7 dBm   | 4 dBm   |
 */
typedef enum {
    RF24_PWR_LVL0 = 0,
    RF24_PWR_LVL1,
    RF24_PWR_LVL2,
    RF24_PWR_LVL3,
    RF24_PWR_MAX = RF24_PWR_LVL3
} rf24_power_e;

struct rf24_configuration
{
    uint8_t payload_len; /* Effective lengths range from 1 to 32, 
                            0 means using dynamic payload length */
    uint8_t addr_width;
    uint8_t channel; /* Channel 0 - 127 */
    rf24_datarate_e data_rate;
    rf24_power_e power_level;
    rt_bool_t power_ext; /* RF_PWR has 3 bits */
    uint16_t retry_delay; /* Auto Retransmit Delay */
    uint8_t repeat_cnt; /* Auto Retransmit Count */
};

struct rf24_device;

struct nrf24_callback
{
    void (*rx_ind)(struct rf24_device *dev, uint8_t *data, uint8_t len, int pipe);
    void (*tx_done)(struct rf24_device *dev, rt_bool_t status);
};

struct rf24_device
{
    struct rt_spi_device *spi_dev; /* SPI obj */
    rt_base_t ce_pin;
    rt_base_t irq_pin;
    rt_sem_t irq_sem;
    struct rf24_configuration config;
    struct nrf24_callback cb;
    uint8_t status;
    rf24_mode_e mode; /* transmit/receive mode */
};

#define RF24_PIN_INVALID    -1

#define rf24_max(a, b)  ((a)>(b)?(a):(b))
#define rf24_min(a, b)  ((a)<(b)?(a):(b))

#define NRF24L01_CONFIG_DEFAULT     \
{                                   \
    .payload_len = 0,               \
    .addr_width = 5,                \
    .channel = 76,                  \
    .data_rate = RF24_1MBPS,        \
    .power_level = RF24_PWR_LVL1,   \
    .power_ext = RT_FALSE,          \
    .retry_delay = 1500,            \
    .repeat_cnt = 15                \
}

#define SI24R1_CONFIG_DEFAULT       \
{                                   \
    .payload_len = 0,               \
    .addr_width = 5,                \
    .channel = 76,                  \
    .data_rate = RF24_1MBPS,        \
    .power_level = RF24_PWR_LVL1,   \
    .power_ext = RT_TRUE,           \
    .retry_delay = 1500,            \
    .repeat_cnt = 15                \
}

struct rf24_device *rf24_device_create(const char *bus_name,
                                               const char *device_name,
                                               rt_base_t csn_pin,
                                               rt_base_t ce_pin,
                                               rt_base_t irq_pin);

void rf24_config(struct rf24_device *dev, struct rf24_configuration *cfg);

void rf24_attach_callback(struct rf24_device *dev, const struct nrf24_callback *cb);

void rf24_set_addr_width(struct rf24_device *dev, uint8_t width);

void rf24_set_retries(struct rf24_device *dev, uint16_t delay, uint8_t count);

void rf24_set_rf_params(struct rf24_device *dev, rf24_datarate_e rate, rf24_power_e level, rt_bool_t extension);

void rf24_set_data_rate(struct rf24_device *dev, rf24_datarate_e data_rate);

void rf24_set_power(struct rf24_device *dev, rf24_power_e level, rt_bool_t extension);

void rf24_set_tx_addr(struct rf24_device *dev, uint8_t *addr, uint8_t len);

void rf24_set_rx_addr(struct rf24_device *dev, uint8_t pipe, uint8_t *addr, uint8_t len);

rt_bool_t rf24_check_available(struct rf24_device *dev);

void rf24_set_to_rx_mode(struct rf24_device *dev);

void rf24_set_to_tx_mode(struct rf24_device *dev);

void rf24_set_channel(struct rf24_device *dev, uint8_t ch);

void rf24_power_up(struct rf24_device *dev);

void rf24_power_down(struct rf24_device *dev);

rf24_mode_e rf24_current_mode(struct rf24_device *dev);

rt_err_t rf24_transmit_packet(struct rf24_device *dev, uint8_t *tx_buf, uint8_t len);

rt_bool_t rf24_is_data_ready(struct rf24_device *dev);

uint8_t rf24_get_data(struct rf24_device *dev, uint8_t *data);

void rf24_dump_reg(struct rf24_device *dev);

void rf24_run(struct rf24_device *dev);

#endif /* __NRF24L01_H_ */
