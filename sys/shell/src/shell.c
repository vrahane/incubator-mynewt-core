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

#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "sysinit/sysinit.h"
#include "syscfg/syscfg.h"
#include "defs/error.h"
#include "console/console.h"
#include "console/prompt.h"
#include "console/ticks.h"
#include "os/os.h"
#include "os/endian.h"
#include "base64/base64.h"
#include "crc/crc16.h"
#include "shell/shell.h"
#include "shell_priv.h"

static shell_nlip_input_func_t g_shell_nlip_in_func;
static void *g_shell_nlip_in_arg;

static struct os_mqueue g_shell_nlip_mq;

#define SHELL_HELP_PER_LINE     6
#define SHELL_MAX_ARGS          20

static int shell_echo_cmd(int argc, char **argv);
static int shell_help_cmd(int argc, char **argv);
int shell_prompt_cmd(int argc, char **argv);
int shell_ticks_cmd(int argc, char **argv);


static void shell_event_console_rdy(struct os_event *ev);

/* Shared queue that the shell uses for work items. */
static struct os_eventq *shell_evq;

static struct shell_cmd g_shell_echo_cmd = {
    .sc_cmd = "echo",
    .sc_cmd_func = shell_echo_cmd
};
static struct shell_cmd g_shell_help_cmd = {
    .sc_cmd = "?",
    .sc_cmd_func = shell_help_cmd
};
static struct shell_cmd g_shell_prompt_cmd = {
   .sc_cmd = "prompt",
   .sc_cmd_func = shell_prompt_cmd
}; 
static struct shell_cmd g_shell_ticks_cmd = {
   .sc_cmd = "ticks",
   .sc_cmd_func = shell_ticks_cmd
};
static struct shell_cmd g_shell_os_tasks_display_cmd = {
    .sc_cmd = "tasks",
    .sc_cmd_func = shell_os_tasks_display_cmd
};
static struct shell_cmd g_shell_os_mpool_display_cmd = {
    .sc_cmd = "mempools",
    .sc_cmd_func = shell_os_mpool_display_cmd
};
static struct shell_cmd g_shell_os_date_cmd = {
    .sc_cmd = "date",
    .sc_cmd_func = shell_os_date_cmd
};

static struct os_event shell_console_rdy_ev = {
    .ev_cb = shell_event_console_rdy,
};

static struct os_mutex g_shell_cmd_list_lock;

static char *shell_line;
static int shell_line_len;
static char *argv[SHELL_MAX_ARGS];

static STAILQ_HEAD(, shell_cmd) g_shell_cmd_list =
    STAILQ_HEAD_INITIALIZER(g_shell_cmd_list);

static struct os_mbuf *g_nlip_mbuf;
static uint16_t g_nlip_expected_len;

static struct os_eventq *
shell_evq_get(void)
{
    return shell_evq;
}

void
shell_evq_set(struct os_eventq *evq)
{
    os_eventq_designate(&shell_evq, evq, NULL);
}

int
shell_cmd_list_lock(void)
{
    int rc;

    if (!os_started()) {
        return (0);
    }

    rc = os_mutex_pend(&g_shell_cmd_list_lock, OS_WAIT_FOREVER);
    if (rc != 0) {
        goto err;
    }
    return (0);
err:
    return (rc);
}

int
shell_cmd_list_unlock(void)
{
    int rc;

    if (!os_started()) {
        return (0);
    }

    rc = os_mutex_release(&g_shell_cmd_list_lock);
    if (rc != 0) {
        goto err;
    }
    return (0);
err:
    return (rc);
}

static int
shell_cmd_find(char *cmd_name, struct shell_cmd **out_cmd)
{
    struct shell_cmd *sc;
    int rc;

    rc = shell_cmd_list_lock();
    if (rc != 0) {
        return rc;
    }

    STAILQ_FOREACH(sc, &g_shell_cmd_list, sc_next) {
        if (!strcmp(sc->sc_cmd, cmd_name)) {
            break;
        }
    }

    rc = shell_cmd_list_unlock();
    if (rc != 0) {
        return rc;
    }

    if (out_cmd != NULL) {
        *out_cmd = sc;
    }

    if (sc == NULL) {
        return SYS_ENOENT;
    }

    return 0;
}

int
shell_cmd_register(struct shell_cmd *sc)
{
    int rc;

#if MYNEWT_VAL(SHELL_DEBUG)
    /* Ensure command not already registered. */
    assert(shell_cmd_find(sc->sc_cmd, NULL) == SYS_ENOENT);
#endif

    /* Add the command that is being registered. */
    rc = shell_cmd_list_lock();
    if (rc != 0) {
        goto err;
    }

    STAILQ_INSERT_TAIL(&g_shell_cmd_list, sc, sc_next);

    rc = shell_cmd_list_unlock();
    if (rc != 0) {
        goto err;
    }

    return (0);
err:
    return (rc);
}

static int
shell_cmd(char *cmd, char **argv, int argc)
{
    struct shell_cmd *sc = NULL;
    int rc;

    rc = shell_cmd_find(cmd, &sc);
    switch (rc) {
    case 0:
        sc->sc_cmd_func(argc, argv);
        return 0;

    case SYS_ENOENT:
        console_printf("Unknown command %s\n", cmd);
        return 0;

    default:
        return rc;
    }
}

static int
shell_process_command(char *line, int len)
{
    char *tok;
    char *tok_ptr;
    int argc;

    tok_ptr = NULL;
    tok = strtok_r(line, " ", &tok_ptr);
    argc = 0;
    while (argc < SHELL_MAX_ARGS - 1 && tok != NULL) {
        argv[argc++] = tok;

        tok = strtok_r(NULL, " ", &tok_ptr);
    }

    /* Terminate the argument list with a null pointer. */
    argv[argc] = NULL;

    if (argc) {
        (void) shell_cmd(argv[0], argv, argc);
    }
    else {
        console_printf("\n");
    }
    console_print_prompt();
    return (0);
}

static int
shell_nlip_process(char *data, int len)
{
    uint16_t copy_len;
    int rc;
    struct os_mbuf *m;
    uint16_t crc;

    rc = base64_decode(data, data);
    if (rc < 0) {
        goto err;
    }
    len = rc;

    if (g_nlip_mbuf == NULL) {
        if (len < 2) {
            rc = -1;
            goto err;
        }

        g_nlip_expected_len = ntohs(*(uint16_t *) data);
        g_nlip_mbuf = os_msys_get_pkthdr(g_nlip_expected_len, 0);
        if (!g_nlip_mbuf) {
            rc = -1;
            goto err;
        }

        data += sizeof(uint16_t);
        len -= sizeof(uint16_t);
    }

    copy_len = min(g_nlip_expected_len - OS_MBUF_PKTHDR(g_nlip_mbuf)->omp_len,
            len);

    rc = os_mbuf_copyinto(g_nlip_mbuf, OS_MBUF_PKTHDR(g_nlip_mbuf)->omp_len,
            data, copy_len);
    if (rc != 0) {
        goto err;
    }

    if (OS_MBUF_PKTHDR(g_nlip_mbuf)->omp_len == g_nlip_expected_len) {
        if (g_shell_nlip_in_func) {
            crc = CRC16_INITIAL_CRC;
            for (m = g_nlip_mbuf; m; m = SLIST_NEXT(m, om_next)) {
                crc = crc16_ccitt(crc, m->om_data, m->om_len);
            }
            if (crc == 0 && g_nlip_expected_len >= sizeof(crc)) {
                os_mbuf_adj(g_nlip_mbuf, -sizeof(crc));
                g_shell_nlip_in_func(g_nlip_mbuf, g_shell_nlip_in_arg);
            } else {
                os_mbuf_free_chain(g_nlip_mbuf);
            }
        } else {
            os_mbuf_free_chain(g_nlip_mbuf);
        }
        g_nlip_mbuf = NULL;
        g_nlip_expected_len = 0;
    }

    return (0);
err:
    return (rc);
}

static int
shell_nlip_mtx(struct os_mbuf *m)
{
#define SHELL_NLIP_MTX_BUF_SIZE (12)
    uint8_t readbuf[SHELL_NLIP_MTX_BUF_SIZE];
    char encodebuf[BASE64_ENCODE_SIZE(SHELL_NLIP_MTX_BUF_SIZE)];
    char pkt_seq[3] = { '\n', SHELL_NLIP_PKT_START1, SHELL_NLIP_PKT_START2 };
    char esc_seq[2] = { SHELL_NLIP_DATA_START1, SHELL_NLIP_DATA_START2 };
    uint16_t totlen;
    uint16_t dlen;
    uint16_t off;
    uint16_t crc;
    int rb_off;
    int elen;
    uint16_t nwritten;
    uint16_t linelen;
    int rc;
    struct os_mbuf *tmp;
    void *ptr;

    /* Convert the mbuf into a packet.
     *
     * starts with 06 09
     * base64 encode:
     *  - total packet length (uint16_t)
     *  - data
     *  - crc
     * base64 encoded data must be less than 122 bytes per line to
     * avoid overflows and adhere to convention.
     *
     * continuation packets are preceded by 04 20 until the entire
     * buffer has been sent.
     */
    crc = CRC16_INITIAL_CRC;
    for (tmp = m; tmp; tmp = SLIST_NEXT(tmp, om_next)) {
        crc = crc16_ccitt(crc, tmp->om_data, tmp->om_len);
    }
    crc = htons(crc);
    ptr = os_mbuf_extend(m, sizeof(crc));
    if (!ptr) {
        rc = -1;
        goto err;
    }
    memcpy(ptr, &crc, sizeof(crc));

    totlen = OS_MBUF_PKTHDR(m)->omp_len;
    nwritten = 0;
    off = 0;

    /* Start a packet */
    console_write(pkt_seq, sizeof(pkt_seq));

    linelen = 0;

    rb_off = 2;
    dlen = htons(totlen);
    memcpy(readbuf, &dlen, sizeof(dlen));

    while (totlen > 0) {
        dlen = min(SHELL_NLIP_MTX_BUF_SIZE - rb_off, totlen);

        rc = os_mbuf_copydata(m, off, dlen, readbuf + rb_off);
        if (rc != 0) {
            goto err;
        }
        off += dlen;

        /* If the next packet will overwhelm the line length, truncate
         * this line.
         */
        if (linelen +
                BASE64_ENCODE_SIZE(min(SHELL_NLIP_MTX_BUF_SIZE - rb_off,
                        totlen - dlen)) >= 120) {
            elen = base64_encode(readbuf, dlen + rb_off, encodebuf, 1);
            console_write(encodebuf, elen);
            console_write("\n", 1);
            console_write(esc_seq, sizeof(esc_seq));
            linelen = 0;
        } else {
            elen = base64_encode(readbuf, dlen + rb_off, encodebuf, 0);
            console_write(encodebuf, elen);
            linelen += elen;
        }

        rb_off = 0;

        nwritten += elen;
        totlen -= dlen;
    }

    elen = base64_pad(encodebuf, linelen);
    console_write(encodebuf, elen);

    console_write("\n", 1);

    return (0);
err:
    return (rc);
}

int
shell_nlip_input_register(shell_nlip_input_func_t nf, void *arg)
{
    g_shell_nlip_in_func = nf;
    g_shell_nlip_in_arg = arg;

    return (0);
}

int
shell_nlip_output(struct os_mbuf *m)
{
    int rc;

    rc = os_mqueue_put(&g_shell_nlip_mq, shell_evq_get(), m);
    if (rc != 0) {
        goto err;
    }

    return (0);
err:
    return (rc);
}

static void
shell_read_console(void)
{
    int rc;
    int full_line;

    while (1) {
        rc = console_read(shell_line + shell_line_len,
          MYNEWT_VAL(SHELL_MAX_INPUT_LEN) - shell_line_len, &full_line);
        if (rc <= 0 && !full_line) {
            break;
        }
        shell_line_len += rc;
        if (full_line) {
            if (shell_line_len > 2) {
                if (shell_line[0] == SHELL_NLIP_PKT_START1 &&
                        shell_line[1] == SHELL_NLIP_PKT_START2) {
                    if (g_nlip_mbuf) {
                        os_mbuf_free_chain(g_nlip_mbuf);
                        g_nlip_mbuf = NULL;
                    }
                    g_nlip_expected_len = 0;

                    rc = shell_nlip_process(&shell_line[2], shell_line_len - 2);
                } else if (shell_line[0] == SHELL_NLIP_DATA_START1 &&
                        shell_line[1] == SHELL_NLIP_DATA_START2) {
                    rc = shell_nlip_process(&shell_line[2], shell_line_len - 2);
                } else {
                    shell_process_command(shell_line, shell_line_len);
                }
            } else {
                shell_process_command(shell_line, shell_line_len);
            }
            shell_line_len = 0;
        }
    }
}

static void
shell_event_console_rdy(struct os_event *ev)
{
    shell_read_console();
}

static void
shell_event_data_in(struct os_event *ev)
{
    struct os_mbuf *m;

    /* Copy data out of the mbuf 12 bytes at a time and write it to
     * the console.
     */
    while (1) {
        m = os_mqueue_get(&g_shell_nlip_mq);
        if (!m) {
            break;
        }

        (void) shell_nlip_mtx(m);

        os_mbuf_free_chain(m);
    }
}

/**
 * This function is called from the console APIs when data is available
 * to be read.  This is either a full line, or when the
 * console buffer (default = 128) is full.
 */
static void
shell_console_rx_cb(void)
{
    os_eventq_put(shell_evq_get(), &shell_console_rdy_ev);
}

static int
shell_echo_cmd(int argc, char **argv)
{
    int i;

    for (i = 1; i < argc; i++) {
        console_write(argv[i], strlen(argv[i]));
        console_write(" ", sizeof(" ")-1);
    }
    console_write("\n", sizeof("\n")-1);

    return (0);
}

static int
shell_help_cmd(int argc, char **argv)
{
    int rc;
    int i = 0;
    struct shell_cmd *sc;

    rc = shell_cmd_list_lock();
    if (rc != 0) {
        return -1;
    }
    console_printf("Commands:\n");
    STAILQ_FOREACH(sc, &g_shell_cmd_list, sc_next) {
        console_printf("%9s ", sc->sc_cmd);
        if (i++ % SHELL_HELP_PER_LINE == SHELL_HELP_PER_LINE - 1) {
            console_printf("\n");
        }
    }
    if (i % SHELL_HELP_PER_LINE) {
        console_printf("\n");
    }
    shell_cmd_list_unlock();

    return (0);
}

void
shell_init(void)
{
    /* Ensure this function only gets called by sysinit. */
    SYSINIT_ASSERT_ACTIVE();

#if !MYNEWT_VAL(SHELL_TASK)
    return;
#endif

    int rc;

    free(shell_line);
    shell_line = NULL;

#if MYNEWT_VAL(SHELL_MAX_INPUT_LEN) > 0
    shell_line = malloc(MYNEWT_VAL(SHELL_MAX_INPUT_LEN));
    SYSINIT_PANIC_ASSERT(shell_line != NULL);
#endif

    rc = os_mutex_init(&g_shell_cmd_list_lock);
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = shell_cmd_register(&g_shell_echo_cmd);
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = shell_cmd_register(&g_shell_help_cmd);
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = shell_cmd_register(&g_shell_prompt_cmd);
    SYSINIT_PANIC_ASSERT(rc == 0);
    
    rc = shell_cmd_register(&g_shell_ticks_cmd);
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = shell_cmd_register(&g_shell_os_tasks_display_cmd);
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = shell_cmd_register(&g_shell_os_mpool_display_cmd);
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = shell_cmd_register(&g_shell_os_date_cmd);
    SYSINIT_PANIC_ASSERT(rc == 0);

    os_mqueue_init(&g_shell_nlip_mq, shell_event_data_in, NULL);
    console_init(shell_console_rx_cb);

    shell_evq_set(os_eventq_dflt_get());
}
