/*******************************************************************************
 * @file    app_remote.c
 * @author  lcc
 * @version
 * @date    2022-07-06
 * @brief   Receive and process remote control data.
 ******************************************************************************/
#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>
#include "nrf24l01.h"

#include "nb_common.h"
#include "nb_types.h"

#include "app_common.h"

#define DBG_TAG "app.comm"
#define DBG_LVL DBG_INFO //  GLOBAL_DBG_LVL
#include <rtdbg.h>

struct msg_header
{
    uint16_t sn;
    uint8_t len;
    uint8_t opcode;
} __attribute__ ((packed));

struct msg_keymap
{
    struct msg_header head;
    uint16_t keymap;
    uint8_t sum;
} __attribute__ ((packed));

struct msg_joystick
{
    struct msg_header head;
    int16_t throttle;
    int16_t yaw;
    int16_t roll;
    int16_t pitch;
    uint8_t sum;
} __attribute__ ((packed));

struct msg_status
{
    struct msg_header head;
    uint16_t status;
    uint8_t sum;
} __attribute__ ((packed));

#define RF24_CSN_PIN    GET_PIN(B, 12)
#define RF24_CE_PIN     GET_PIN(A, 12)
#define RF24_IRQ_PIN    GET_PIN(A, 11)
#define SPI_BUS         "spi2"

static uint8_t my_addr[5] = {'n', 'e', 'w', 'b', 'e'};
static uint8_t peer_addr[5] = {'c', 'n', 't', 'r', 'l'};

struct rf24_device *si24r1 = NULL;

uint16_t rx_sn;
uint16_t tx_sn = 0;

rt_bool_t comm_ok = RT_TRUE;
#define COMM_FAILED_LIMIT 10

static rt_bool_t _check_crc(const void *data, uint8_t len)
{
    int i;
    uint8_t sum = 0;
    const uint8_t *p = data;

    RT_ASSERT(data != NULL);

    if (len <= 1)
        return RT_FALSE;

    for (i = 0; i < len - 1; i++)
    {
        sum += p[i];
    }

    return (sum == p[len - 1]);
}

static uint8_t calc_sum(void *buf, int len)
{
    uint8_t sum = 0;
    uint8_t *p = buf;
    for (int i = 0; i < len; i++)
    {
        sum += p[i];
    }
    return sum;
}

struct remote_control rc_in;

static void _process_remote_joystick(struct msg_joystick *msg)
{
    rc_in.throttle =    msg->throttle;
    rc_in.yaw =         msg->yaw;
    rc_in.roll =        msg->roll;
    rc_in.pitch =       msg->pitch;
#if DBG_LVL >= 2//DBG_LOG
    rt_kprintf("thro %d, yaw %d, roll %d, pitch %d\n", rc_in.throttle, rc_in.yaw,
               rc_in.roll, rc_in.pitch);
#endif
}

void nb_remote_control_update(struct remote_control *rc)
{
    rc->throttle = rc_in.throttle;
    rc->yaw = rc_in.yaw;
    rc->roll = rc_in.roll;
    rc->pitch = rc_in.pitch;
}

rt_bool_t nb_remote_comm_ok(void)
{
    return comm_ok;
}

enum key_index
{
    KEY_WKUP = 0,
    KEY_1,
    KEY_UP,
    KEY_DOWN,
    KEY_LEFT,
    KEY_RIGHT,
    KEY_IDX_MAX
};

static void _process_remote_key(struct msg_keymap *msg)
{
    uint16_t keymap = msg->keymap;
    struct msg_status status;
#if DBG_LVL >= DBG_LOG
    rt_kprintf("keymap %04x\n", keymap);
#endif
    if (keymap & BIT(KEY_1))
    {
        uint16_t status = nb_control_status_get();
        if (status & BIT(UAV_STATUS_FLIGTH_UNLOCK_OFT))
        {
            /* lock airplane */
            status &= ~BIT(UAV_STATUS_FLIGTH_UNLOCK_OFT);
            nb_control_status_set(status);
            LOG_D("airplane locked");
        }
        else
        {
            /* unlock */
            status |= BIT(UAV_STATUS_FLIGTH_UNLOCK_OFT);
            nb_control_status_set(status);
            LOG_D("airplane unlocked");
        }
    }
    if (keymap & BIT(KEY_UP))
        ;
    if (keymap & BIT(KEY_DOWN))
        ;
    if (keymap & BIT(KEY_LEFT))
        ;
    if (keymap & BIT(KEY_RIGHT))
        ;

    status.head.sn = tx_sn++;
    status.head.len = sizeof(status);
    status.head.opcode = 0x80;
    status.status = nb_control_status_get();
    status.sum = calc_sum(&status, sizeof(status) - 1);
    rf24_set_to_tx_mode(si24r1);
    rf24_transmit_packet(si24r1, (void *)&status, sizeof(status));
    rf24_set_to_rx_mode(si24r1);
}

static void rf24_rx_hdl(struct rf24_device *dev, uint8_t *data, uint8_t len, int pipe)
{
#if DBG_LVL >= DBG_LOG
    // for debug...
    for (int i = 0; i < len; i++)
        rt_kprintf("%02x ", data[i]);
    rt_kprintf("\n");
#endif
    if (_check_crc(data, len))
    {
        struct msg_header *head = (void *)data;
        rx_sn = head->sn;
        switch (head->opcode)
        {
        case 0x01:
            _process_remote_joystick((void *)data);
            break;
        case 0x02:
            _process_remote_key((void *)data);
            break;
        case 0x00:
            break;
        default:
            break;
        }
    }
    else {
#if DBG_LVL >= DBG_LOG
        rt_kprintf("rf24: sum error\n");
#endif
    }
}

static const struct nrf24_callback _cb = {
    .rx_ind = rf24_rx_hdl,
};

void nb_remote_comm_thread_entry(void *parameter)
{
    /* configuration */
    struct rf24_configuration cfg = SI24R1_CONFIG_DEFAULT;
    cfg.power_ext = RT_TRUE;
    cfg.channel = 52;
    cfg.data_rate = RF24_250KBPS;
    cfg.power_level = RF24_PWR_LVL1;
    cfg.payload_len = 0; // 0 means dynamic length
    rf24_config(si24r1, &cfg);

    rf24_attach_callback(si24r1, &_cb);

    rf24_set_rx_addr(si24r1, 1, my_addr, 5);
    rf24_set_tx_addr(si24r1, peer_addr, 5);

    rf24_set_to_rx_mode(si24r1);
    LOG_D("set to rx mode");
    while (1)
    {
        rf24_run(si24r1);
        // rt_thread_mdelay(1000);
    }
}

void nb_remote_comm_check_thread_entry(void *parameter)
{
    uint32_t failed_cnt = 0;
    uint16_t last_sn = rx_sn;
    while (1)
    {
        //rf24_dump_reg(si24r1);
        if (nb_control_status_get() & BIT(UAV_STATUS_FLIGTH_UNLOCK_OFT))
        {
            if (last_sn == rx_sn)
            {
                LOG_W("lost communication!");
                if (failed_cnt < COMM_FAILED_LIMIT)
                    failed_cnt++;
            }
            else
            {
                failed_cnt = 0;
                comm_ok = RT_TRUE;
            }
            if (failed_cnt == COMM_FAILED_LIMIT)
            {
                if (comm_ok != RT_FALSE)
                {
                    rf24_dump_reg(si24r1);
                    // emergency landing!!
                    comm_ok = RT_FALSE;
                }
            }
        }
        last_sn = rx_sn;
        rt_thread_mdelay(100);
    }
}

static int remote_app_init(void)
{
    rt_thread_t tid1, tid2;

    si24r1 = rf24_device_create(SPI_BUS, "rf24_tx",
                                RF24_CSN_PIN, RF24_CE_PIN, RF24_IRQ_PIN);
    if (RT_NULL == si24r1)
    {
        LOG_E("failed to create a nrf24 device.");
        return -1;
    }

    rt_thread_mdelay(500);
    while (!rf24_check_available(si24r1))
    {
        LOG_E("nrf24: device not available.");
        rt_thread_mdelay(1000);
    }

    LOG_E("nrf24: init ok.");

    tid1 = rt_thread_create("comm_thread", nb_remote_comm_thread_entry, RT_NULL,
                            2048,    /* stack size */
                            25,      /* priority */
                            5);      /* time slice */
    if (RT_NULL == tid1)
    {
        return -1;
    }
    rt_thread_startup(tid1);

    tid2 = rt_thread_create("comm_check_thread", nb_remote_comm_check_thread_entry,
                            RT_NULL,
                            1024,    /* stack size */
                            26,      /* priority */
                            5);      /* time slice */
    if (RT_NULL == tid2)
    {
        return -1;
    }

    rt_thread_startup(tid2);
    return 0;
}

INIT_APP_EXPORT(remote_app_init);
