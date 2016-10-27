/**
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <assert.h>
#include <string.h>
#include "syscfg/syscfg.h"
#include "sysinit/sysinit.h"
#include "bsp/bsp.h"
#include "os/os.h"

#include "hal/hal_gpio.h"
#include "mgmt/mgmt.h"
#include "uart/uart.h"

/* BLE */
#include "nimble/ble.h"
#include "controller/ble_ll.h"
#include "host/ble_hs.h"

/* RAM HCI transport. */
#include "transport/ram/ble_hci_ram.h"

/* RAM persistence layer. */
#include "store/ram/ble_store_ram.h"

/* Mandatory services. */
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

/* Application-specified header. */
#include "blecent.h"

#include "newtmgr/newtmgr.h"

extern struct nmgr_cbuf nmgr_task_cbuf;

static int cmd;
static uint16_t xoff;
static uint16_t yoff;


/** Log data. */
struct log blecent_log;

/* For LED toggling */
static int g_led_pin;

/** blecent task settings. */
#define BLECENT_TASK_PRIO           1
#define BLECENT_STACK_SIZE          (OS_STACK_ALIGN(336))

struct os_eventq blecent_evq;
struct os_task blecent_task;
bssnz_t os_stack_t blecent_stack[BLECENT_STACK_SIZE];

/** Our global device address (public) */
uint8_t g_dev_addr[BLE_DEV_ADDR_LEN] = {0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x0a};

/** Our random address (in case we need it) */
uint8_t g_random_addr[BLE_DEV_ADDR_LEN];

static int blecent_gap_event(struct ble_gap_event *event, void *arg);

struct ble_gap_conn_entry {
    uint8_t addr_type;
    uint8_t addr[6];
    uint8_t connected;
    uint16_t conn_handle;
};

struct ble_gap_conn_entry peer_white_list[] =
    {
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x01}, 0},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x02}, 0},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x03}, 0},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x04}, 0},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x05}, 0},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x06}, 0},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x07}, 0},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x08}, 0},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x09}, 0},
     {BLE_ADDR_TYPE_PUBLIC, {0x0b, 0x0a, 0x0b, 0x0b, 0x00, 0x10}, 0}
    };

/**
 * Initiates the GAP general discovery procedure.
 */
static int
blecent_scan(void)
{
    struct ble_gap_disc_params disc_params;
    int rc;

    /* Tell the controller to filter duplicates; we don't want to process
     * repeated advertisements from the same device.
     */
    disc_params.filter_duplicates = 1;

    /**
     * Perform a passive scan.  I.e., don't send follow-up scan requests to
     * each advertiser.
     */
    disc_params.passive = 1;

    /* Use defaults for the rest of the parameters. */
    disc_params.itvl = 0;
    disc_params.window = 0;
    disc_params.filter_policy = 0;
    disc_params.limited = 0;

    rc = ble_gap_disc(BLE_ADDR_TYPE_PUBLIC, BLE_HS_FOREVER, &disc_params,
                      blecent_gap_event, NULL);
    if (rc != 0) {
        BLECENT_LOG(ERROR, "Error initiating GAP discovery procedure; rc=%d\n",
                    rc);
    }

    return rc;
}

/**
 * Indicates whether we should tre to connect to the sender of the specified
 * advertisement.  The function returns a positive result if the device
 * advertises connectability and support for the Alert Notification service.
 */
static int
blecent_should_connect(const struct ble_gap_disc_desc *disc)
{
    int i;

    /* The device has to be advertising connectability. */
    if (disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_ADV_IND &&
        disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_DIR_IND) {

        return 0;
    }

    /* The device has to advertise support for the Alert Notification
     * service (0x1811).
     */
    for (i = 0; i < disc->fields->num_uuids16; i++) {
        if (disc->fields->uuids16[i] == BLECENT_SVC_ALERT_UUID) {
            return 1;
        }
    }

    return 0;
}

/**
 * Connects to the sender of the specified advertisement of it looks
 * interesting.  A device is "interesting" if it advertises connectability and
 * support for the Alert Notification service.
 */
static void
blecent_connect_if_interesting(const struct ble_gap_disc_desc *disc)
{
    int rc;
    int i;
    
    /* Don't do anything if we don't care about this advertiser. */
    if (!blecent_should_connect(disc)) {
        return;
    }

    /* Scanning must be stopped before a connection can be initiated. */
    rc = ble_gap_disc_cancel();
    if (rc != 0) {
        BLECENT_LOG(DEBUG, "Failed to cancel scan; rc=%d\n", rc);
        return;
    }

    /* Try to connect the the advertiser.  Allow 30 seconds (30000 ms) for
     * timeout.
     */

    for (i=0; i<10; i++) {

        if (!memcmp(peer_white_list[i].addr, disc->addr, 6) && peer_white_list[i].connected != 1 && cmd == 2) {

            ble_gap_connect(BLE_ADDR_TYPE_PUBLIC, BLE_HCI_CONN_PEER_ADDR_PUBLIC, peer_white_list[i].addr,
                            30000, NULL, blecent_gap_event, NULL);
            peer_white_list[i].connected = 1;
        }
    }
}

int
nmgr_create_req(struct os_mbuf **req)
{
    struct nmgr_hdr *req_hdr;
    int rc;

    rc = 0;
    req_hdr = NULL;

    *req = os_msys_get_pkthdr(512, OS_MBUF_USRHDR_LEN(*req));
    if (!*req) {
        rc = os_mbuf_copydata(*req, 0, sizeof(*req_hdr), req_hdr);
        if (rc < 0) {
            assert(0);
            goto err;
        }
    }

    req_hdr = nmgr_init_req(*req, req_hdr);
    req_hdr->nh_id = NMGR_OP_READ;
    req_hdr->nh_group = MGMT_GROUP_ID_IMGDEMO;
    if (!req_hdr) {
        assert(0);
        rc = MGMT_ERR_ENOMEM;
        goto err;
    }

    CborError g_err = CborNoError;
    CborEncoder *penc = &nmgr_task_cbuf.n_b.encoder;
    CborEncoder req_cbor;
    g_err |= cbor_encoder_create_map(penc, &req_cbor, CborIndefiniteLength);
    g_err |= cbor_encode_text_stringz(&req_cbor, "x");
    g_err |= cbor_encode_int(&req_cbor, g_dev_addr[5]%3);
    //g_err |= cbor_encode_text_stringz(&req_cbor, "y");
    //g_err |= cbor_encode_int(&req_cbor, yoff);
    g_err |= cbor_encoder_close_container(penc, &req_cbor);

    req_hdr->nh_len += cbor_encode_bytes_written(&nmgr_task_cbuf.n_b.encoder);

    //req_hdr->nh_flags |= NMGR_F_CBOR_RSP_COMPLETE;
    req_hdr->nh_len = htons(req_hdr->nh_len);
    req_hdr->nh_group = htons(req_hdr->nh_group);

err:
    return rc;
}

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that is
 * established.  blecent uses the same callback for all connections.
 *
 * @param event                 The event being signalled.
 * @param arg                   Application-specified argument; unused by
 *                                  blecent.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
static int
blecent_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    int rc;
    int rand_val;
    int i;

    xoff = 0;
    yoff = 0;

    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        /* An advertisment report was received during GAP discovery. */
        print_adv_fields(event->disc.fields);

        /* Try to connect to the advertiser if it looks interesting. */
        blecent_connect_if_interesting(&event->disc);
        return 0;

    case BLE_GAP_EVENT_CONNECT:

        /* A new connection was established or a connection attempt failed. */
        if (event->connect.status == 0) {
            /* Connection successfully established. */
            BLECENT_LOG(INFO, "Connection established ");

            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            assert(rc == 0);
            print_conn_desc(&desc);
            BLECENT_LOG(INFO, "\n");

            if (peer_empty()) {
                /* Image of 320 * 240 size */
                rand_val = rand();
                xoff = rand_val%(320/4) * 4;
                yoff = rand_val%(240/3) * 3;
            } else {
                xoff = (xoff+1)%(320/4) * 4;
                yoff = (yoff+1)%(240/3) * 3;
            }

            for (i=0; i<10; i++) {
                if (!memcmp(peer_white_list[i].addr, desc.peer_id_addr, 6) && cmd == 2) {
                    peer_white_list[i].connected = 1;
                    peer_white_list[i].conn_handle =  desc.conn_handle;
                }
            }

            /* Remember peer. */
            rc = peer_add(event->connect.conn_handle);
            if (rc != 0) {
                BLECENT_LOG(ERROR, "Failed to add peer; rc=%d\n", rc);
                return 0;
            }

        } else {
            /* Connection attempt failed; resume scanning. */
            BLECENT_LOG(ERROR, "Error: Connection failed; status=%d\n",
                        event->connect.status);
        }

        rc = blecent_scan();
        assert(rc == 0);
        /* Try to connect the the advertiser.  Allow 30 seconds (30000 ms) for
         * timeout.
         */
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        /* Connection terminated. */
        BLECENT_LOG(INFO, "disconnect; reason=%d ", event->disconnect.reason);
        print_conn_desc(&event->disconnect.conn);
        BLECENT_LOG(INFO, "\n");

        for (i=0; i<10; i++) {
            if (!memcmp(peer_white_list[i].addr, event->disconnect.conn.peer_id_addr, 6)) {
                peer_white_list[i].connected = 0;
                peer_white_list[i].conn_handle = 0;
            }
        }

        /* Forget about peer. */
        peer_delete(event->disconnect.conn.conn_handle);
        /* Try to connect the the advertiser.  Allow 30 seconds (30000 ms) for
         * timeout.
         */
        rc = blecent_scan();
        assert(rc == 0 || rc == BLE_HS_EBUSY || rc == BLE_HS_EALREADY);
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        /* Encryption has been enabled or disabled for this connection. */
        BLECENT_LOG(INFO, "encryption change event; status=%d ",
                    event->enc_change.status);
        rc = ble_gap_conn_find(event->enc_change.conn_handle, &desc);
        assert(rc == 0);
        print_conn_desc(&desc);
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX:
        /* Peer sent us a notification or indication. */
        BLECENT_LOG(INFO, "received %s; conn_handle=%d attr_handle=%d "
                          "attr_len=%d\n",
                    event->notify_rx.indication ?
                        "indication" :
                        "notification",
                    event->notify_rx.conn_handle,
                    event->notify_rx.attr_handle,
                    OS_MBUF_PKTLEN(event->notify_rx.om));

        /* Attribute data is contained in event->notify_rx.attr_data. */
        return 0;

    case BLE_GAP_EVENT_MTU:
        BLECENT_LOG(INFO, "mtu update event; conn_handle=%d cid=%d mtu=%d\n",
                    event->mtu.conn_handle,
                    event->mtu.channel_id,
                    event->mtu.value);
        return 0;

    default:
        return 0;
    }
}

static void
blecent_on_reset(int reason)
{
    BLECENT_LOG(ERROR, "Resetting state; reason=%d\n", reason);
}

static void
blecent_on_sync(void)
{
    /* Begin scanning for a peripheral to connect to. */
    blecent_scan();
}

static struct os_callout_func blescan_timer;


static void
blescan_tmo(void *unused)
{
    int rc = 0;
    if (unused != (void *)&blecent_evq) {
        return;
    }
    blecent_scan();
    rc = os_callout_reset(&blescan_timer.cf_c, OS_TICKS_PER_SEC/2);
    assert(rc == 0);
}

static char uart_rx_buf[15];
static char uart_tx_buf[128];
static int uart_rx_idx;
static int uart_tx_idx;
static int cmd_len;
static struct uart_dev *udv;

/*
 * Called by UART driver to send out next character.
 *
 * Interrupts disabled when nmgr_uart_tx_char/nmgr_uart_rx_char are called.
 */
static int
app_uart_tx_char(void *arg)
{
    if (uart_tx_idx < cmd_len) {
        return uart_tx_buf[uart_tx_idx++];
    } else {
        return -1;
    }
}

/*
 * Receive a character from UART.
 */
static int
app_uart_rx_char(void *arg, uint8_t data)
{
    int rc;
    struct os_mbuf *m_req;
    int i;

    rc = 0;
    m_req = NULL;
    uart_rx_buf[uart_rx_idx++] = data;
    if (uart_rx_idx >= 7) {
        uart_rx_idx = 0;
        //memset(uart_rx_buf, 0, 7);
        cmd = uart_rx_buf[2];
        if (cmd == 3) {
            rc = nmgr_create_req(&m_req);
            assert(m_req);
            for (i=0; i<10; i++) {
                /* Using hardcoded nmgr attribute handle */
                if (peer_white_list[i].connected) {
                    rc = ble_gattc_write_no_rsp(peer_white_list[i].conn_handle, 18, m_req);
                }
            }
        }
    }

    rc = 0;
    return rc;
}

static void
app_uart_init(void)
{
    struct uart_conf uc = {
        .uc_speed = 9600,
        .uc_databits = 8,
        .uc_stopbits = 1,
        .uc_parity = UART_PARITY_NONE,
        .uc_flow_ctl = UART_FLOW_CTL_NONE,
        .uc_tx_char = app_uart_tx_char,
        .uc_rx_char = app_uart_rx_char,
        .uc_cb_arg = NULL
    };

    udv = (struct uart_dev *)os_dev_open("uart0", 0, &uc);
    uart_rx_idx = 0;
}

/**
 * Event loop for the main blecent task.
 */
static void
blecent_task_handler(void *unused)
{
    struct os_event *ev;
    struct os_callout_func *cf;
    int rc;

    /* Activate the host.  This causes the host to synchronize with the
     * controller.
     */
    rc = ble_hs_start();
    assert(rc == 0);

    srand(0);

    /* Set the led pin for the devboard */
    g_led_pin = LED_BLINK_PIN;
    hal_gpio_init_out(g_led_pin, 1);
#if defined(BSP_nrf51_blenano)||defined(BSP_nrf52dk)
    hal_gpio_clear(g_led_pin);
#else
    hal_gpio_set(g_led_pin);
#endif
    /* XXXXXXX Hacky stuff */
    app_uart_init();

    os_callout_func_init(&blescan_timer, &blecent_evq,
                         blescan_tmo, &blecent_evq);
    rc = os_callout_reset(&blescan_timer.cf_c, OS_TICKS_PER_SEC/2);
    assert(rc == 0);

    while (1) {
        ev = os_eventq_get(&blecent_evq);
        switch (ev->ev_type) {
        case OS_EVENT_T_TIMER:
            cf = (struct os_callout_func *)ev;
            assert(cf->cf_func);
            cf->cf_func(CF_ARG(cf));
            break;

        default:
            assert(0);
            break;
        }
    }
}

/**
 * main
 *
 * The main function for the project. This function initializes the os, calls
 * init_tasks to initialize tasks (and possibly other objects), then starts the
 * OS. We should not return from os start.
 *
 * @return int NOTE: this function should never return!
 */
int
main(void)
{
    int rc;

    /* Initialize OS */
    sysinit();

    /* Initialize the blecent log. */
    log_register("blecent", &blecent_log, &log_console_handler, NULL);

    /* Initialize the eventq for the application task. */
    os_eventq_init(&blecent_evq);

    /* Create the blecent task.  All application logic and NimBLE host
     * operations are performed in this task.
     */
    os_task_init(&blecent_task, "blecent", blecent_task_handler,
                 NULL, BLECENT_TASK_PRIO, OS_WAIT_FOREVER,
                 blecent_stack, BLECENT_STACK_SIZE);

    /* Configure the host. */
    ble_hs_cfg.parent_evq = &blecent_evq;
    ble_hs_cfg.reset_cb = blecent_on_reset;
    ble_hs_cfg.sync_cb = blecent_on_sync;
    ble_hs_cfg.store_read_cb = ble_store_ram_read;
    ble_hs_cfg.store_write_cb = ble_store_ram_write;

    /* Initialize data structures to track connected peers. */
    rc = peer_init(MYNEWT_VAL(BLE_MAX_CONNECTIONS), 64, 64, 64);
    assert(rc == 0);

    /* Set the default device name. */
    rc = ble_svc_gap_device_name_set("nimble-blecent");
    assert(rc == 0);

    /* Start the OS */
    os_start();

    /* os start should never return. If it does, this should be an error */
    assert(0);

    return 0;
}
