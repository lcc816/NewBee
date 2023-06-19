/*******************************************************************************
 * @file     nrf24l01.c
 * @author   lcc
 * @version  
 * @date     2022-02-13
 * @brief    
 ******************************************************************************/

#include "board.h"
#include "nrf24l01.h"
#include <drv_spi.h>
#include <string.h>

#ifdef __STM32F1XX_H
#include "nrf24l01_stm32_adapter.h"
#else
#define nrf24l01_spi_csn_config(x)
#endif

#define DBG_TAG     "drv.rf24"
#define DBG_LVL     DBG_WARNING //DBG_LOG
#include <rtdbg.h>

#define BIT(n)          (1<<(n))
#define BITS(m, n)      (~(BIT(m)-1) & ((BIT(n) - 1) | BIT(n)))

/*----------------------------------------------------------------------------*/
/*                     The following are static functions                     */
/*----------------------------------------------------------------------------*/

static void _spi_write_reg(struct rf24_device *dev, uint8_t addr, uint8_t data)
{
    uint8_t buffer[2] = {addr | RF24_CMD_W_REGISTER, data};

    dev->status = 0xFF;

    if (rt_spi_transfer(dev->spi_dev, buffer, buffer, 2) != 2)
        return;

    dev->status = buffer[0];
}

static uint8_t _spi_read_reg(struct rf24_device *dev, uint8_t addr)
{
    /* send 1 byte then receive 1 byte */
    uint8_t buffer[2] = {addr | RF24_CMD_R_REGISTER, 0xFF};

    dev->status = 0xFF;

    if (rt_spi_transfer(dev->spi_dev, buffer, buffer, 2) != 2)
        return 0xFF;

    dev->status = buffer[0];
    return buffer[1];
}

static void _spi_write_buffer(struct rf24_device *dev, uint8_t addr, uint8_t *buffer, uint8_t bytes)
{
    struct rt_spi_message msg1, msg2;
    addr |= RF24_CMD_W_REGISTER;
    msg1.send_buf   = &addr;
    msg1.recv_buf   = &dev->status;
    msg1.length     = 1;
    msg1.cs_take    = 1;
    msg1.cs_release = 0;
    msg1.next       = &msg2;

    msg2.send_buf   = buffer;
    msg2.recv_buf   = RT_NULL;
    msg2.length     = bytes;
    msg2.cs_take    = 0;
    msg2.cs_release = 1;
    msg2.next       = RT_NULL;
    rt_spi_transfer_message(dev->spi_dev, &msg1);
}

static void _spi_read_buffer(struct rf24_device *dev, uint8_t addr, uint8_t *buffer, uint8_t bytes)
{
    struct rt_spi_message msg1, msg2;
    addr |= RF24_CMD_R_REGISTER;
    msg1.send_buf   = &addr;
    msg1.recv_buf   = &dev->status;
    msg1.length     = 1;
    msg1.cs_take    = 1;
    msg1.cs_release = 0;
    msg1.next       = &msg2;

    msg2.send_buf   = RT_NULL;
    msg2.recv_buf   = buffer;
    msg2.length     = bytes;
    msg2.cs_take    = 0;
    msg2.cs_release = 1;
    msg2.next       = RT_NULL;
    rt_spi_transfer_message(dev->spi_dev, &msg1);
}

static void _ce_low(struct rf24_device *dev)
{
    rt_pin_write(dev->ce_pin, PIN_LOW);
}

static void _ce_high(struct rf24_device *dev)
{
    rt_pin_write(dev->ce_pin, PIN_HIGH);
}

static void _flush_tx(struct rf24_device *dev)
{
    uint8_t cmd = RF24_CMD_FLUSH_TX;

    rt_spi_transfer(dev->spi_dev, &cmd, &dev->status, 1);
}

static void _flush_rx(struct rf24_device *dev)
{
    uint8_t cmd = RF24_CMD_FLUSH_RX;

    rt_spi_transfer(dev->spi_dev, &cmd, &dev->status, 1);
}

static uint8_t _parse_data_rate_reg(rf24_datarate_e data_rate)
{
    /*
     * Encoding:
     * [RF_DR_LOW, RF_DR_HIGH]:
     * ‘00’ – 1Mbps
     * ‘01’ – 2Mbps
     * ‘10’ – 250kbps
     * ‘11’ – Reserved
     */
    if (data_rate == RF24_250KBPS)
        return BIT(RF24_SHIFT_RF_DR_LOW);
    else if (data_rate == RF24_2MBPS)
        return BIT(RF24_SHIFT_RF_DR_HIGH);
    else
        return 0;
}

static uint8_t _parse_power_reg(rf24_power_e level, rt_bool_t extension)
{
    /*
     * | level (enum value) | nRF24L01 | Si24R1 with extension bit = 1 | Si24R1 with extension bit = 0 |
     * |:------------------:|:-------:|:--------:|:-------:|
     * |   RF24_PWR_LVL0    | -18 dBm |  -6 dBm  | -12 dBm |
     * |   RF24_PWR_LVL1    | -12 dBm |  -0 dBm  | -4 dBm  |
     * |   RF24_PWR_LVL2    | -6 dBm  |  3 dBm   | 1 dBm   |
     * |   RF24_PWR_LVL3    |  0 dBm  |  7 dBm   | 4 dBm   |
     */
    if (level > RF24_PWR_MAX)
        level = RF24_PWR_MAX;
    return extension ? (level << 1) + 1 : (level << 1);
}

static void _clear_status_flags(struct rf24_device *dev)
{
    // Clear the flags in STATUS register
    _spi_write_reg(dev, RF24_REG_STATUS, BIT(RF24_SHIFT_RX_DR) |
                   BIT(RF24_SHIFT_TX_DS) | BIT(RF24_SHIFT_MAX_RT));
}

static void _transmit_with_padding(struct rf24_device *dev, uint8_t *tx_buf,
                                uint8_t len, uint8_t padding_len)
{
    struct rt_spi_message msg1, msg2, msg3;
    uint8_t pad[32] = {0};
    uint8_t cmd = RF24_CMD_W_TX_PAYLOAD;

    msg1.send_buf   = &cmd;
    msg1.recv_buf   = &dev->status;
    msg1.length     = 1;
    msg1.cs_take    = 1;
    msg1.cs_release = 0;
    msg1.next       = &msg2;

    msg2.send_buf   = tx_buf;
    msg2.recv_buf   = RT_NULL;
    msg2.length     = len;
    msg2.cs_take    = 1;
    msg2.cs_release = 0;
    msg2.next       = &msg3;

    msg3.send_buf   = pad;
    msg3.recv_buf   = RT_NULL;
    msg3.length     = padding_len;
    msg3.cs_take    = 0;
    msg3.cs_release = 1;
    msg3.next       = RT_NULL;

    rt_spi_transfer_message(dev->spi_dev, &msg1);
}

static uint8_t _data_len(struct rf24_device *dev)
{
    RT_ASSERT(dev != NULL);

    if (!dev->config.payload_len)
    {
        uint8_t len;
        uint8_t cmd = RF24_CMD_R_RX_PL_WID;

        rt_spi_send_then_recv(dev->spi_dev, &cmd, 1, &len, 1);
        return len;
    }
    else
    {
        return dev->config.payload_len;
    }
}

static void _irq_handler(void *args)
{
    struct rf24_device *dev = (struct rf24_device *)args;
    if (dev->irq_sem != RT_NULL)
    {
        rt_sem_release(dev->irq_sem);
        LOG_D("rf24 release a semaphore");
    }
}

/*----------------------------------------------------------------------------*/
/*                  The following are the external functions                  */
/*----------------------------------------------------------------------------*/
void rf24_set_addr_width(struct rf24_device *dev, uint8_t width)
{
    RT_ASSERT(dev != NULL);

    /*
     * '00' - Illegal
     * '01' - 3 bytes
     * '10' - 4 bytes
     * '11' – 5 bytes
     **/
    dev->config.addr_width = rf24_max(3, rf24_min(5, width));

    _spi_write_reg(dev, RF24_REG_SETUP_AW, dev->config.addr_width - 2);
}

void rf24_set_retries(struct rf24_device *dev, uint16_t delay, uint8_t count)
{
    RT_ASSERT(dev != NULL);

    /*
     * '0000' - 250uS
     * '0001' - 500uS
     * '0010' - 750uS
     * ...
     * '1111' – 40000uS
     **/
    uint8_t ard = rf24_min(15, (delay / 250));
    dev->config.repeat_cnt = rf24_min(15, count);
    dev->config.retry_delay = ard * 250;
    _spi_write_reg(dev, RF24_REG_SETUP_RETR, (ard << RF24_SHIFT_ARD) | dev->config.repeat_cnt);
}

void rf24_set_rf_params(struct rf24_device *dev, rf24_datarate_e rate, rf24_power_e level, rt_bool_t extension)
{
    RT_ASSERT(dev != NULL);

    dev->config.data_rate = rate;
    dev->config.power_level = level;
    uint8_t setup = _parse_data_rate_reg(rate);
    setup |= _parse_power_reg(level, extension);
    _spi_write_reg(dev, RF24_REG_RF_SETUP, setup);
}

void rf24_set_data_rate(struct rf24_device *dev, rf24_datarate_e data_rate)
{
    RT_ASSERT(dev != NULL);

    uint8_t setup = _spi_read_reg(dev, RF24_REG_RF_SETUP);
    setup &= (uint8_t)~(BIT(RF24_SHIFT_RF_DR_HIGH) | BIT(RF24_SHIFT_RF_DR_LOW));
    setup |= _parse_data_rate_reg(data_rate);

    _spi_write_reg(dev, RF24_REG_RF_SETUP, setup);
}

void rf24_set_power(struct rf24_device *dev, rf24_power_e level, rt_bool_t extension)
{
    RT_ASSERT(dev != NULL);

    uint8_t setup = _spi_read_reg(dev, RF24_REG_RF_SETUP) & (uint8_t)~0x3;
    setup |= _parse_power_reg(level, extension);

    _spi_write_reg(dev, RF24_REG_RF_SETUP, setup);
}

void rf24_set_channel(struct rf24_device *dev, uint8_t channel)
{
    RT_ASSERT(dev != NULL);

    dev->config.channel = rf24_min(125, channel);
    _spi_write_reg(dev, RF24_REG_RF_CH, dev->config.channel);
}

void rf24_power_up(struct rf24_device *dev)
{
    RT_ASSERT(dev != NULL);

    uint8_t value = _spi_read_reg(dev, RF24_REG_CONFIG);
    if(!(value & BIT(RF24_SHIFT_PWR_UP)))
    {
        value |= BIT(RF24_SHIFT_PWR_UP);
        _spi_write_reg(dev, RF24_REG_CONFIG, value);
        rt_thread_mdelay(3);
    }
}

void rf24_power_down(struct rf24_device *dev)
{
    RT_ASSERT(dev != NULL);

    _ce_low(dev);
    _spi_write_reg(dev, RF24_REG_CONFIG, _spi_read_reg(dev, RF24_REG_CONFIG) & ~BIT(RF24_SHIFT_PWR_UP));
}

#define RT_NRF24_DEFAULT_SPI_CFG                            \
{                                                           \
    .mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB,     \
    .data_width = 8,                                        \
    .max_hz = 8 * 1000 * 1000,                              \
}

struct rf24_device *rf24_device_create(const char *bus_name,
                                       const char *device_name,
                                       rt_base_t csn_pin,
                                       rt_base_t ce_pin,
                                       rt_base_t irq_pin)
{
    RT_ASSERT(bus_name != NULL);
    RT_ASSERT(device_name != NULL);

    rt_err_t result;
    struct rt_spi_device *spi_device;
    struct rt_spi_configuration cfg = RT_NRF24_DEFAULT_SPI_CFG;
#ifdef __STM32F1XX_H
    struct stm32_hw_spi_cs *stm32_cs_pin;
    const struct stm32_pin_index *index;
#else
    rt_base_t *cs_pin = NULL; // may modify for porting
#endif

    struct rf24_device *rf24_dev = NULL;

    /* initialize the cs pin && select the slave*/
    //nrf24l01_spi_csn_config(csn_pin);
    rt_pin_mode(csn_pin, PIN_MODE_OUTPUT);
    rt_pin_write(csn_pin, PIN_HIGH);

    /* attach the device to spi bus*/
    spi_device = (struct rt_spi_device *)rt_malloc(sizeof(struct rt_spi_device));
    if (spi_device == NULL)
    {
        LOG_E("alloc rt_spi_device failed");
        goto error0;
    }

#ifdef __STM32F1XX_H
    stm32_cs_pin = (struct stm32_hw_spi_cs *)rt_malloc(sizeof(struct stm32_hw_spi_cs));
    if (stm32_cs_pin == NULL)
    {
        LOG_E("alloc cs_pin failed");
        goto error1;
    }
    index = get_stm32_pin(csn_pin);
    stm32_cs_pin->GPIOx = index->gpio;
    stm32_cs_pin->GPIO_Pin = index->pin;
    result = rt_spi_bus_attach_device(spi_device, device_name, bus_name, (void *)stm32_cs_pin);
#else
    result = rt_spi_bus_attach_device(spi_device, device_name, bus_name, NULL);
#endif
    if (result != RT_EOK)
    {
        LOG_E("%s attach to %s faild, %d", device_name, bus_name, result);
        goto error2;
    }
    rt_spi_configure(spi_device, &cfg);
    LOG_D("%s attach to %s done", device_name, bus_name);

    rf24_dev = (struct rf24_device *)rt_malloc(sizeof(struct rf24_device));
    if (rf24_dev == NULL)
    {
        LOG_E("alloc nrf24l01_device failed");
        goto error2;
    }

    rf24_dev->spi_dev = spi_device;

    rt_pin_mode(ce_pin, PIN_MODE_OUTPUT);
    rt_pin_write(ce_pin, PIN_LOW);
    rf24_dev->ce_pin = ce_pin;

    if (irq_pin != RF24_PIN_INVALID)
    {
        rf24_dev->irq_sem = rt_sem_create("rf24_irq", 0, RT_IPC_FLAG_FIFO);
        if (rf24_dev->irq_sem == NULL)
        {
            LOG_E("failed to alloc irq sem");
            goto error3;
        }
        LOG_D("created semaphore rf24_irq");
        rt_pin_mode(irq_pin, PIN_MODE_INPUT_PULLUP);
        rt_pin_attach_irq(irq_pin, PIN_IRQ_MODE_FALLING, _irq_handler, rf24_dev);
        rt_pin_irq_enable(irq_pin, PIN_IRQ_ENABLE);
    }

    rf24_dev->mode = RF24_MODE_POWROFF;

    return rf24_dev;

error3:
    rt_free(rf24_dev);
error2:
#ifdef __STM32F1XX_H
    rt_free(stm32_cs_pin);
#else
    ;
#endif
error1:
    rt_free(spi_device);
error0:
    return NULL;
}

void rf24_config(struct rf24_device *dev, struct rf24_configuration *cfg)
{
    struct rf24_configuration *config = &dev->config;
    int DYNPD = 0, FEATURE = 0;

    RT_ASSERT(dev != NULL);

    /* set configuration */
    memcpy(config, cfg, sizeof(struct rf24_configuration));

    if (config->payload_len)
    {
        /* Sets static payload length of pipe x */
        _spi_write_reg(dev, RF24_REG_RX_PW_P0, config->payload_len);
        _spi_write_reg(dev, RF24_REG_RX_PW_P1, config->payload_len);
    }
    else
    {
        /* Enable dynamic payload length data pipe 0 */
        DYNPD = BIT(RF24_SHIFT_DPL_P0)|BIT(RF24_SHIFT_DPL_P1);
    }

    _spi_write_reg(dev, RF24_REG_DYNPD, DYNPD);
    if (DYNPD)
    {
        /* Enables Dynamic Payload Length */
        FEATURE |= BIT(RF24_SHIFT_EN_DPL);
    }
    _spi_write_reg(dev, RF24_REG_FEATRUE, FEATURE);
    /* Enable auto-ack on all pipes */
    _spi_write_reg(dev, RF24_REG_EN_AA, 0x3F);
    /* Enable Rx data pipe 0 & 1 */
    _spi_write_reg(dev, RF24_REG_EN_RXADDR, BIT(RF24_SHIFT_ERX_P0) | BIT(RF24_SHIFT_ERX_P1));

    rf24_set_addr_width(dev, cfg->addr_width);

    rf24_set_retries(dev, cfg->retry_delay, cfg->repeat_cnt);

    rf24_set_channel(dev, cfg->channel);

    rf24_set_rf_params(dev, cfg->data_rate, cfg->power_level, cfg->power_ext);

    _clear_status_flags(dev);
    _flush_rx(dev);
    _flush_tx(dev);

    _spi_write_reg(dev, RF24_REG_CONFIG, 
                   BIT(RF24_SHIFT_EN_CRC) | BIT(RF24_SHIFT_CRCO));
    rf24_power_up(dev);

    dev->mode = RF24_MODE_UNCONF;
}

void rf24_attach_callback(struct rf24_device *dev, const struct nrf24_callback *cb)
{
    RT_ASSERT(dev != NULL);
    RT_ASSERT(cb != NULL);

    uint8_t status = _spi_read_reg(dev, RF24_REG_CONFIG);

    if (cb->rx_ind != NULL)
    {
        dev->cb.rx_ind = cb->rx_ind;
        status &= ~BIT(RF24_SHIFT_MASK_RX_DR);
    }
    if (cb->tx_done != NULL)
    {
        dev->cb.tx_done = cb->tx_done;
        status &= ~(BIT(RF24_SHIFT_MASK_TX_DS) | BIT(RF24_SHIFT_MASK_MAX_RT));
    }
    _spi_write_reg(dev, RF24_REG_CONFIG, status);
}

void rf24_set_tx_addr(struct rf24_device *dev, uint8_t *addr, uint8_t len)
{
    RT_ASSERT(dev != NULL);
    RT_ASSERT(addr != NULL);

    len = (len > 5) ? 5 : len;
    _spi_write_buffer(dev, RF24_REG_TX_ADDR, addr, len);
    /*
     * RX_ADDR_P0 must be set to the sending addr for auto ack to work.
     */
    _spi_write_buffer(dev, RF24_REG_RX_ADDR_P0, addr, len);
}

/*
  Only RX_ADDR_P0 and RX_ADDR_P1 have a width of 5 bytes
  RX_ADDR_P2~5 only have LSB
  and the MSBytes of RX_ADDR_P2~5 are equal to RX_ADDR_P1[39:8]
*/
void rf24_set_rx_addr(struct rf24_device *dev, uint8_t pipe, uint8_t *addr, uint8_t len)
{
    RT_ASSERT(dev != NULL);
    RT_ASSERT(addr != NULL);

    len = (len > 5) ? 5 : len;
    pipe = (pipe > 5) ? 5 : pipe;

    _spi_write_buffer(dev, RF24_REG_RX_ADDR_P0 + pipe, addr, len);
}

rt_bool_t rf24_check_available(struct rf24_device *dev)
{
    RT_ASSERT(dev != NULL);

    uint8_t i;
    uint8_t tx_addr[5] = {'A', 'B', 'C', 'D', 'E'};
    uint8_t read_buf[5] = {0};
    uint8_t backup_addr[5] = {0};
    _spi_read_buffer(dev, RF24_REG_TX_ADDR, backup_addr, 5);

    _spi_write_buffer(dev, RF24_REG_TX_ADDR, tx_addr, 5);
    _spi_read_buffer(dev, RF24_REG_TX_ADDR, read_buf, 5);
    for (i = 0; i < 5; i++)
    {
        if (tx_addr[i] != read_buf[i])
        {
            return RT_FALSE;
        }
    }

    _spi_write_buffer(dev, RF24_REG_TX_ADDR, backup_addr, 5);
    return RT_TRUE;
}

void rf24_set_to_rx_mode(struct rf24_device *dev)
{
    uint8_t config_reg = _spi_read_reg(dev, RF24_REG_CONFIG);

    RT_ASSERT(dev != NULL);

    config_reg |= BIT(RF24_SHIFT_PRIM_RX);
    _spi_write_reg(dev, RF24_REG_CONFIG, config_reg);
    _clear_status_flags(dev);
    _ce_high(dev);

    dev->mode = RF24_MODE_RX;
}

void rf24_set_to_tx_mode(struct rf24_device *dev)
{
    uint8_t config_reg = _spi_read_reg(dev, RF24_REG_CONFIG);

    RT_ASSERT(dev != NULL);

    _ce_low(dev);
    rt_hw_us_delay(100);
    config_reg &= ~BIT(RF24_SHIFT_PRIM_RX);
    _spi_write_reg(dev, RF24_REG_CONFIG, config_reg);
    // enable rx pip0 for auto ack
    _spi_write_reg(dev, RF24_REG_EN_RXADDR, _spi_read_reg(dev, RF24_REG_EN_RXADDR) | BIT(RF24_SHIFT_ERX_P0));

    dev->mode = RF24_MODE_TX;
}

rf24_mode_e rf24_current_mode(struct rf24_device *dev)
{
    RT_ASSERT(dev != NULL);

    return dev->mode;
}

rt_err_t rf24_transmit_packet(struct rf24_device *dev, uint8_t *tx_buf, uint8_t len)
{
    uint8_t status;
    uint8_t timeout;

    uint8_t padding_len = 0;

    RT_ASSERT(dev != NULL);
    RT_ASSERT(tx_buf != NULL);

    if (dev->mode != RF24_MODE_TX)
        return STATUS_FAIL;

    _flush_tx(dev);
    _clear_status_flags(dev);
    /* go to standby-I mode */
    _ce_low(dev);

    if (len > 32)
        len = 32;
    if (dev->config.payload_len)
    {
        if (len > dev->config.payload_len)
            len = dev->config.payload_len;
        padding_len = dev->config.payload_len - len;
    }

    /* Command: W_TX_PAYLOAD */
    //rt_spi_send_then_send(dev->spi_dev, &cmd, 1, tx_buf, len);
    _transmit_with_padding(dev, tx_buf, len, padding_len);

    // Start transmission
    _ce_high(dev);

    timeout = 30;
    do
    {
        status = _spi_read_reg(dev, RF24_REG_STATUS);
        /* Maximum number of TX retransmits reached */
        if (status & BIT(RF24_SHIFT_MAX_RT))
        {
            _flush_tx(dev);
            /* Clear interrupt to enable further communication */
            _spi_write_reg(dev, RF24_REG_STATUS, BIT(RF24_SHIFT_MAX_RT));
            return STATUS_MAX_RT;
        }
        if (status & BIT(RF24_SHIFT_TX_DS))
        {
            return STATUS_OK;
        }
        rt_hw_us_delay(100);
    } while (timeout--);

    return STATUS_TIMEOUT;
}

/* data is ready when: 1) RX_DR is asserted,
 * 2) RX_DR is not asserted but RX FIFO is not empty.
 */
rt_bool_t rf24_is_data_ready(struct rf24_device *dev)
{
    RT_ASSERT(dev != NULL);

    if (_spi_read_reg(dev, RF24_REG_STATUS) & BIT(RF24_SHIFT_RX_DR))
        return RT_TRUE;

    if (_spi_read_reg(dev, RF24_REG_FIFO_STATUS) & BIT(RF24_SHIFT_RX_EMPTY))
        return RT_FALSE;

    return RT_TRUE;
}

uint8_t rf24_get_data(struct rf24_device *dev, uint8_t *data)
{
    uint8_t len;
    uint8_t cmd;

    RT_ASSERT(dev != NULL);

    len = _data_len(dev);
    // Flush RX FIFO if the read value is larger than 32 bytes
    if (len > 32)
    {
        len = 0;
        _flush_rx(dev);
        goto ret;
    }

    /* read rx payload */
    cmd = RF24_CMD_R_RX_PAYLOAD;
    rt_spi_send_then_recv(dev->spi_dev, &cmd, 1, data, len);

ret:
    /* Clear RX_DR flag */
    _spi_write_reg(dev, RF24_REG_STATUS, BIT(RF24_SHIFT_RX_DR));
    return len;
}

void rf24_dump_reg(struct rf24_device *dev)
{
    uint8_t value, buf[5] = {0};
    for (uint8_t i = 0; i <= 0x1D; i++)
    {
        if (i == 0xa || i == 0xb || i == 0x10)
        {
            _spi_read_buffer(dev, i, buf, 5);
            rt_kprintf("%X: %X %X %X %X %X\n", i,
                       buf[0], buf[1], buf[2], buf[3], buf[4]);
        }
        else
        {
            value = _spi_read_reg(dev, i);
            rt_kprintf("%X: %X\n", i, value);
        }
    }
}

void rf24_run(struct rf24_device *dev)
{
    uint8_t status;

    if (dev->irq_sem != NULL)
    {
        LOG_D("wait for interrupt");
        rt_sem_take(dev->irq_sem, 1000);
    }

    status = _spi_read_reg(dev, RF24_REG_STATUS);

    if (status & BIT(RF24_SHIFT_RX_DR))
    {
        uint8_t pipe = (status >> RF24_SHIFT_RX_P_NO) & 0x7;
        uint8_t data[32];
        uint8_t len = rf24_get_data(dev, data);
        if (len > 0 && dev->cb.rx_ind != NULL)
        {
            LOG_D("received %u bytes", len);
            dev->cb.rx_ind(dev, data, len, pipe);
        }
        _spi_write_reg(dev, RF24_REG_STATUS, BIT(RF24_SHIFT_RX_DR));
    }

    if (status & BIT(RF24_SHIFT_TX_DS))
    {
        if (dev->cb.tx_done != NULL)
        {
            dev->cb.tx_done(dev, RT_TRUE);
        }
        _spi_write_reg(dev, RF24_REG_STATUS, BIT(RF24_SHIFT_TX_DS));
    }

    if (status & BIT(RF24_SHIFT_MAX_RT))
    {
        if (dev->cb.tx_done != NULL)
        {
            dev->cb.tx_done(dev, RT_FALSE);
        }
        _spi_write_reg(dev, RF24_REG_STATUS, BIT(RF24_SHIFT_MAX_RT));
    }
}
