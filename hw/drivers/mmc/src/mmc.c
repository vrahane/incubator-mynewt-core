/*
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

#include <hal/hal_spi.h>
#include <hal/hal_gpio.h>

#include <mmc/mmc.h>

#include <stdio.h>

#define MIN(n, m) (((n) < (m)) ? (n) : (m))

/* Current used MMC commands */
#define CMD0                (0)            /* GO_IDLE_STATE */
#define CMD1                (1)            /* SEND_OP_COND (MMC) */
#define CMD8                (8)            /* SEND_IF_COND */
#define CMD12               (12)           /* STOP_TRANSMISSION */
#define CMD16               (16)           /* SET_BLOCKLEN */
#define CMD17               (17)           /* READ_SINGLE_BLOCK */
#define CMD18               (18)           /* READ_MULTIPLE_BLOCK */
#define CMD24               (24)           /* WRITE_BLOCK */
#define CMD25               (25)           /* WRITE_MULTIPLE_BLOCK */
#define CMD55               (55)           /* APP_CMD */
#define CMD58               (58)           /* READ_OCR */
#define ACMD41              (0x80 + 41)    /* SEND_OP_COND (SDC) */

/* Response types */
#define R1                  (0)
#define R1b                 (1)
#define R2                  (2)
#define R3                  (3)
#define R7                  (4)

/* Response status */
#define R_IDLE              (0x01)
#define R_ERASE_RESET       (0x02)
#define R_ILLEGAL_COMMAND   (0x04)
#define R_CRC_ERROR         (0x08)
#define R_ERASE_ERROR       (0x10)
#define R_ADDR_ERROR        (0x20)
#define R_PARAM_ERROR       (0x40)

#define BLOCK_LEN           (512)

static uint8_t g_block_buf[BLOCK_LEN];

static struct hal_spi_settings mmc_settings = {
    .data_order = HAL_SPI_MSB_FIRST,
    .data_mode  = HAL_SPI_MODE0,
    /* XXX: MMC initialization accepts clocks in the range 100-400KHz */
    .baudrate   = 100,
    .word_size  = HAL_SPI_WORD_SIZE_8BIT,
};

static int g_spi_num = -1;
static int g_ss_pin = -1;

/**
 * TODO: switch to high-speed SPI after initialization.
 */

static int
mmc_error_by_status(uint8_t status)
{
    return MMC_OK;
}

static int8_t
response_type_by_cmd(uint8_t cmd)
{
    switch (cmd) {
        case CMD8   : return R7;
        case CMD58  : return R3;
    }
    return R1;
}

static uint32_t
ocr_from_r3(uint8_t *response)
{
    uint32_t ocr;

    ocr  = (uint32_t) response[3];
    ocr |= (uint32_t) response[2] <<  8;
    ocr |= (uint32_t) response[1] << 16;
    ocr |= (uint32_t) response[0] << 24;

#if 0
    printf("Card supports: ");
    if (ocr & (1 << 15)) {
        printf("2.7-2.8 ");
    }
#endif

    return ocr;
}

static uint8_t
voltage_from_r7(uint8_t *response)
{
    return response[2] & 0xF;
}

static uint8_t
send_mmc_cmd(uint8_t cmd, uint32_t payload)
{
    int n;
    uint8_t response[4];
    uint8_t status;
    uint8_t type;

    if (cmd & 0x80) {
        /* TODO: refactor recursion? */
        send_mmc_cmd(CMD55, 0);
    }

    /* 4.7.2: Command Format */
    hal_spi_tx_val(g_spi_num, 0x40 | (cmd & ~0x80));

    hal_spi_tx_val(g_spi_num, payload >> 24 & 0xff);
    hal_spi_tx_val(g_spi_num, payload >> 16 & 0xff);
    hal_spi_tx_val(g_spi_num, payload >>  8 & 0xff);
    hal_spi_tx_val(g_spi_num, payload       & 0xff);

    /**
     * 7.2.2 Bus Transfer Protection
     *   NOTE: SD is in CRC off mode by default but CMD0 and CMD8 always
     *   require a valid CRC.
     *   NOTE: CRC can be turned on with CMD59 (CRC_ON_OFF).
     */
    if (cmd == CMD0) {
        status = hal_spi_tx_val(g_spi_num, 0x95);
    } else if (cmd == CMD8) {
        status = hal_spi_tx_val(g_spi_num, 0x87);
    } else {
        /* Any value is OK */
        status = hal_spi_tx_val(g_spi_num, 0x01);
    }

    printf("=======================\n");
    printf("sending cmd %d\n", cmd & ~0x80);

    n = 10;
    do {
        status = hal_spi_tx_val(g_spi_num, 0xff);
    } while ((status & 0x80) && --n);

    if (n) {
        type = response_type_by_cmd(cmd);
        printf("status=0x%x\n", status);
        if (!(type == R1 || status & R_ILLEGAL_COMMAND || status & R_CRC_ERROR)) {
            /* Read remaining data for this command */
            for (n = 0; n < sizeof(response); n++) {
                response[n] = (uint8_t) hal_spi_tx_val(g_spi_num, 0xff);
            }

            printf("response=");
            for (n = 0; n < sizeof(response); n++) {
                printf("[%02x]", response[n]);
            }
            printf(" \n");

            switch (type) {
                case R3:
                    /* NOTE: ocr is defined in section 5.1 */
                    printf("ocr=0x%08lx\n", ocr_from_r3(response));
                    break;
                case R7:
                    printf("voltage=0x%x\n", voltage_from_r7(response));
                    break;
            }
        }
    } else {
        printf("status=%x, n=%d\n", status, n);
    }

    printf("\n");

    return status;
}

/**
 * NOTE:
 */

/**
 * Initialize the MMC driver
 *
 * @param spi_num Number of the SPI channel to be used by MMC
 * @param spi_cfg Low-level device specific SPI configuration
 * @param ss_pin Number of SS pin if SW controlled, -1 otherwise
 *
 * @return 0 on success, non-zero on failure
 */
int
mmc_init(int spi_num, void *spi_cfg, int ss_pin)
{
    int rc;
    int i;
    uint8_t status;
    uint32_t ocr;
    os_time_t wait_to;

    g_ss_pin = ss_pin;
    g_spi_num = spi_num;

    if (g_ss_pin != -1) {
        hal_gpio_init_out(g_ss_pin, 1);
    }

    rc = hal_spi_init(g_spi_num, spi_cfg, HAL_SPI_TYPE_MASTER);
    if (rc) {
        return (rc);
    }

    rc = hal_spi_config(g_spi_num, &mmc_settings);
    if (rc) {
        return (rc);
    }

    hal_spi_set_txrx_cb(0, NULL, NULL);
    hal_spi_enable(0);

    /**
     * NOTE: The state machine below follows:
     *       Section 6.4.1: Power Up Sequence for SD Bus Interface.
     *       Section 7.2.1: Mode Selection and Initialization.
     */

    /* give 10ms for VDD rampup */
    os_time_delay(OS_TICKS_PER_SEC / 100);

    hal_gpio_write(g_ss_pin, 0);

    /* send the required >= 74 clock cycles */
    for (i = 0; i < 74; i++) {
        hal_spi_tx_val(0, 0xff);
    }

    /* put card in idle state */
    status = send_mmc_cmd(CMD0, 0);

    if (status != R_IDLE) {
        /* No card inserted or bad card! */
        hal_gpio_write(g_ss_pin, 1);
        return mmc_error_by_status(status);
    }

    /**
     * 4.3.13: Ask for 2.7-3.3V range, send 0xAA pattern
     *
     * NOTE: cards that are not compliant with "Physical Spec Version 2.00"
     *       will answer this with R_ILLEGAL_COMMAND.
     */
    status = send_mmc_cmd(CMD8, 0x1AA);
    if (status & R_ILLEGAL_COMMAND) {
        /* Ver1.x SD Memory Card or Not SD Memory Card */

        ocr = send_mmc_cmd(CMD58, 0);

        /* TODO: check if voltage range is ok! */

        if (ocr & R_ILLEGAL_COMMAND) {

        }

        /* TODO: set blocklen */

    } else {
        /* Ver2.00 or later SD Memory Card */

        /* TODO:
         * 1) check CMD8 response pattern and voltage range.
         * 2) DONE: send ACMD41 while in R_IDLE up to 1s.
         * 3) send CMD58, check CCS in response.
         */

#define TIME_TO_WAIT (3 * OS_TICKS_PER_SEC)

        wait_to = os_time_get() + TIME_TO_WAIT;
        status = send_mmc_cmd(ACMD41, 0x40000000);

        while (status & R_IDLE) {
            if (os_time_get() > wait_to) {
                break;
            }
            os_time_delay(OS_TICKS_PER_SEC / 10);
            status = send_mmc_cmd(ACMD41, 0);
        }

        /* TODO: check CCS = OCR[30] */
        status = send_mmc_cmd(CMD58, 0);
    }

    hal_gpio_write(g_ss_pin, 1);

    return rc;
}

/**
 * @return 0 on success, non-zero on failure
 */
int
mmc_read(uint32_t addr, void *buf, size_t len)
{
    uint8_t cmd;
    uint8_t res;
    uint32_t n;
    size_t block_len;
    size_t block_count;
    os_time_t timeout;
    size_t block_addr;
    size_t offset;
    size_t index;
    size_t amount;

    hal_gpio_write(g_ss_pin, 0);

    block_len = (len + BLOCK_LEN - 1) & ~(BLOCK_LEN - 1);
    block_count = block_len / BLOCK_LEN;
    block_addr = addr / BLOCK_LEN;
    offset = addr - (block_addr * BLOCK_LEN);

    printf("block_addr=%d, offset=%d\n", block_addr, offset);
    printf("block_len=%d, block_count=%d\n", block_len, block_count);

    cmd = (block_count == 1) ? CMD17 : CMD18;
    res = send_mmc_cmd(cmd, block_addr);
    /* TODO: should be 0 */
    printf("res=0x%x\n", res);

    /* 7.3.3 Control tokens */

    /* wait up to 1s (should be 200ms???) for control token */
    timeout = os_time_get() + OS_TICKS_PER_SEC;
    do {
        res = hal_spi_tx_val(g_spi_num, 0xff);
        if (res != 0xFF) {
            break;
        }
        os_time_delay(OS_TICKS_PER_SEC / 20);
    } while (os_time_get() < timeout);

    printf("res=0x%x\n", res);
    if (res == 0xFE) {
        index = 0;
        while (block_count--) {
            /**
             * FIXME: on last run doesn't need to transfer all BLOCK_LEN bytes!
             */

            for (n = 0; n < BLOCK_LEN; n++) {
                g_block_buf[n] = hal_spi_tx_val(g_spi_num, 0xff);
            }

            /* consume CRC-16 */
            hal_spi_tx_val(g_spi_num, 0xff);
            hal_spi_tx_val(g_spi_num, 0xff);

            amount = MIN(BLOCK_LEN - offset, len);

            printf("copying %d bytes to index %d\n", amount, index);
            memcpy(((uint8_t *)buf + index), &g_block_buf[offset], amount);

            offset = 0;
            len -= amount;
            index += amount;
        }

        if (cmd == CMD18) {
            send_mmc_cmd(CMD12, 0);
        }

        hal_gpio_write(g_ss_pin, 1);

        return MMC_OK;
    }

    hal_gpio_write(g_ss_pin, 1);

    return MMC_CARD_ERROR;
}

/**
 * @return 0 on success, non-zero on failure
 */
int
mmc_write(uint32_t addr, void *buf, size_t len)
{
    return 0;
}
