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

#include <os/os.h>
#include <string.h>
#include <stdio.h>

#include "syscfg/syscfg.h"

#if MYNEWT_VAL(LOG_NEWTMGR)

#include "mgmt/mgmt.h"
#include "cborattr/cborattr.h"
#include "tinycbor/cbor_cnt_writer.h"
#include "log/log.h"

/* Source code is only included if the newtmgr library is enabled.  Otherwise
 * this file is compiled out for code size.
 */

static int log_nmgr_read(struct mgmt_cbuf *njb);
static int log_nmgr_clear(struct mgmt_cbuf *njb);
static int log_nmgr_module_list(struct mgmt_cbuf *njb);
static int log_nmgr_level_list(struct mgmt_cbuf *njb);
static int log_nmgr_logs_list(struct mgmt_cbuf *njb);
static struct mgmt_group log_nmgr_group;


/* ORDER MATTERS HERE.
 * Each element represents the command ID, referenced from newtmgr.
 */
static struct mgmt_handler log_nmgr_group_handlers[] = {
    [LOGS_NMGR_OP_READ] = {log_nmgr_read, log_nmgr_read},
    [LOGS_NMGR_OP_CLEAR] = {log_nmgr_clear, log_nmgr_clear},
    [LOGS_NMGR_OP_MODULE_LIST] = {log_nmgr_module_list, NULL},
    [LOGS_NMGR_OP_LEVEL_LIST] = {log_nmgr_level_list, NULL},
    [LOGS_NMGR_OP_LOGS_LIST] = {log_nmgr_logs_list, NULL}
};

/**
 * Log encode entry
 * @param log structure, log_offset, dataptr, len
 * @return 0 on success; non-zero on failure
 */
static int
log_nmgr_encode_entry(struct log *log, struct log_offset *log_offset,
                      void *dptr, uint16_t len)
{
    struct log_entry_hdr ueh;
    char data[128];
    int dlen;
    int rc;
    CborError g_err = CborNoError;
    CborEncoder *penc = (CborEncoder*)log_offset->lo_arg;
    CborEncoder rsp;

    rc = log_read(log, dptr, &ueh, 0, sizeof(ueh));
    if (rc != sizeof(ueh)) {
        rc = OS_ENOENT;
        goto err;
    }
    rc = OS_OK;

    /* If specified timestamp is nonzero, it is the primary criterion, and the
     * specified index is the secondary criterion.  If specified timetsamp is
     * zero, specified index is the only criterion.
     *
     * If specified timestamp == 0: encode entries whose index >=
     *     specified index.
     * Else: encode entries whose timestamp >= specified timestamp and whose
     *      index >= specified index
     */

    if (log_offset->lo_ts == 0) {
        if (log_offset->lo_index > ueh.ue_index) {
            goto err;
        }
    } else if (ueh.ue_ts < log_offset->lo_ts   ||
               (ueh.ue_ts == log_offset->lo_ts &&
                ueh.ue_index < log_offset->lo_index)) {
        goto err;
    }

    dlen = min(len-sizeof(ueh), 128);

    rc = log_read(log, dptr, data, sizeof(ueh), dlen);
    if (rc < 0) {
        rc = OS_ENOENT;
        goto err;
    }
    data[rc] = 0;

    if (log_offset->lo_data_len) {
        cbor_encoder_init_writer(&rsp, &cbor_cnt_writer,
                                 penc->end);
    }
    g_err |= cbor_encoder_create_map(penc, &rsp, CborIndefiniteLength);
    g_err |= cbor_encode_text_stringz(&rsp, "msg");
    g_err |= cbor_encode_text_stringz(&rsp, data);
    g_err |= cbor_encode_text_stringz(&rsp, "ts");
    g_err |= cbor_encode_int(&rsp, ueh.ue_ts);
    g_err |= cbor_encode_text_stringz(&rsp, "level");
    g_err |= cbor_encode_uint(&rsp, ueh.ue_level);
    g_err |= cbor_encode_text_stringz(&rsp, "index");
    g_err |= cbor_encode_uint(&rsp,  ueh.ue_index);
    g_err |= cbor_encode_text_stringz(&rsp, "module");
    g_err |= cbor_encode_uint(&rsp,  ueh.ue_module);
    g_err |= cbor_encoder_close_container(penc, &rsp);

    if (g_err) {
        return MGMT_ERR_ENOMEM;
    }
    return (0);
err:
    return (rc);
}

/**
 * Log encode entries
 * @param log structure, the encoder, timestamp, index
 * @return 0 on success; non-zero on failure
 */
static int
log_encode_entries(struct log *log, CborEncoder *cb, uint8_t cnt,
                   int64_t ts, uint32_t index)
{
    int rc;
    struct log_offset log_offset;
    CborEncoder entries;
    CborError g_err = CborNoError;

    memset(&log_offset, 0, sizeof(log_offset));

    g_err |= cbor_encode_text_stringz(cb, "entries");

    if (cnt) {
        cbor_encoder_init_writer(&entries, &cbor_cnt_writer,
                                 cb->end);
    }

    g_err |= cbor_encoder_create_array(cb, &entries, CborIndefiniteLength);

    log_offset.lo_arg       = &entries;
    log_offset.lo_index     = index;
    log_offset.lo_ts        = ts;

    /*
     * lo_data_len is used to signal if we are counting the number of bytes
     * the encoding would require
     */
    if (cnt) {
        log_offset.lo_data_len  = *(size_t *)cb->end;
    }

    rc = log_walk(log, log_nmgr_encode_entry, &log_offset);

    g_err |= cbor_encoder_close_container(cb, &entries);
    if (g_err) {
        return MGMT_ERR_ENOMEM;
    }

    return rc;
}

/**
 * Log encode function
 * @param log structure, the encoder, json_value,
 *        timestamp, index
 * @return 0 on success; non-zero on failure
 */
static int
log_encode(struct log *log, CborEncoder *cb, uint8_t cnt,
            int64_t ts, uint32_t index)
{
    int rc;
    CborEncoder logs;
    CborError g_err = CborNoError;

    if (cnt) {
        cbor_encoder_init_writer(&logs, &cbor_cnt_writer,
                                 cb->end);
    }

    g_err |= cbor_encoder_create_map(cb, &logs, CborIndefiniteLength);
    g_err |= cbor_encode_text_stringz(&logs, "name");
    g_err |= cbor_encode_text_stringz(&logs, log->l_name);

    g_err |= cbor_encode_text_stringz(&logs, "type");
    g_err |= cbor_encode_uint(&logs, log->l_log->log_type);

    rc = log_encode_entries(log, &logs, cnt, ts, index);
    g_err |= cbor_encoder_close_container(cb, &logs);
    if (g_err) {
        return MGMT_ERR_ENOMEM;
    }
    return rc;
}

static int
log_nmgr_cbor_encode(CborEncoder *enc, uint8_t cnt, uint64_t *index,
                     char *name, int64_t *ts)
{
    CborError g_err = CborNoError;
    CborEncoder logs;
    struct log *log;
    int name_len;
    int rc;

    rc = OS_OK;

    g_err |= cbor_encode_text_stringz(enc, "next_index");
    g_err |= cbor_encode_int(enc, g_log_info.li_next_index);

    g_err |= cbor_encode_text_stringz(enc, "logs");

    if (cnt) {
        cbor_encoder_init_writer(&logs, &cbor_cnt_writer,
                                 enc->end);
    }

    g_err |= cbor_encoder_create_array(enc, &logs,
                                       CborIndefiniteLength);

    name_len = strlen(name);
    log = NULL;
    while (1) {
        log = log_list_get_next(log);
        if (!log) {
            break;
        }

        if (log->l_log->log_type == LOG_TYPE_STREAM) {
            continue;
        }

        /* Conditions for returning specific logs */
        if ((name_len > 0) && strcmp(name, log->l_name)) {
            continue;
        }

        rc = log_encode(log, &logs, cnt, *ts, *index);
        if (rc) {
            goto err;
        }

        /* If a log was found, encode and break */
        if (name_len > 0) {
            break;
        }
    }


    /* Running out of logs list and we have a specific log to look for */
    if (!log && name_len > 0) {
        rc = OS_EINVAL;
    }

err:
    g_err |= cbor_encoder_close_container(enc, &logs);
    g_err |= cbor_encode_text_stringz(enc, "rc");
    g_err |= cbor_encode_int(enc, rc);

    return g_err;
}

/**
 * Newtmgr Log read handler
 * @param cbor buffer
 * @return 0 on success; non-zero on failure
 */
static int
log_nmgr_read(struct mgmt_cbuf *cb)
{
    int rc;
    char name[LOG_NAME_MAX_LEN] = {0};
    int64_t ts;
    uint64_t index;
    CborError g_err = CborNoError;
    CborEncoder cbor_cnt_encoder;
    size_t bytes_written;

    const struct cbor_attr_t attr[4] = {
        [0] = {
            .attribute = "log_name",
            .type = CborAttrTextStringType,
            .addr.string = name,
            .len = sizeof(name)
        },
        [1] = {
            .attribute = "ts",
            .type = CborAttrIntegerType,
            .addr.integer = &ts
        },
        [2] = {
            .attribute = "index",
            .type = CborAttrUnsignedIntegerType,
            .addr.uinteger = &index
        },
        [3] = {
            .attribute = NULL
        }
    };

    rc = cbor_read_object(&cb->it, attr);
    if (rc) {
        return rc;
    }

    /* Initially count and check if we can actually encode the log */
    cbor_encoder_init_writer(&cbor_cnt_encoder, &cbor_cnt_writer,
                             &bytes_written);
    g_err |= log_nmgr_cbor_encode(&cbor_cnt_encoder, 1, &index, name, &ts);
    if (g_err || bytes_written > 400) {
        return MGMT_ERR_ENOMEM;
    }
    g_err |= log_nmgr_cbor_encode(&cb->encoder, 0, &index, name, &ts);
    if (g_err) {
        return MGMT_ERR_ENOMEM;
    }
    rc = 0;
    return (rc);
}

/**
 * Newtmgr Module list handler
 * @param nmgr json buffer
 * @return 0 on success; non-zero on failure
 */
static int
log_nmgr_module_list(struct mgmt_cbuf *cb)
{
    int module;
    char *str;
    CborError g_err = CborNoError;
    CborEncoder modules;

    g_err |= cbor_encode_text_stringz(&cb->encoder, "rc");
    g_err |= cbor_encode_int(&cb->encoder, MGMT_ERR_EOK);

    g_err |= cbor_encode_text_stringz(&cb->encoder, "module_map");
    g_err |= cbor_encoder_create_map(&cb->encoder, &modules,
                                     CborIndefiniteLength);

    module = LOG_MODULE_DEFAULT;
    while (module < LOG_MODULE_MAX) {
        str = LOG_MODULE_STR(module);
        if (!strcmp(str, "UNKNOWN")) {
            module++;
            continue;
        }

        g_err |= cbor_encode_text_stringz(&modules, str);
        g_err |= cbor_encode_uint(&modules, module);
        module++;
    }

    g_err |= cbor_encoder_close_container(&cb->encoder, &modules);

    if (g_err) {
        return MGMT_ERR_ENOMEM;
    }
    return (0);
}

/**
 * Newtmgr Log list handler
 * @param nmgr json buffer
 * @return 0 on success; non-zero on failure
 */
static int
log_nmgr_logs_list(struct mgmt_cbuf *cb)
{
    CborError g_err = CborNoError;
    CborEncoder log_list;
    struct log *log;

    g_err |= cbor_encode_text_stringz(&cb->encoder, "rc");
    g_err |= cbor_encode_int(&cb->encoder, MGMT_ERR_EOK);

    g_err |= cbor_encode_text_stringz(&cb->encoder, "log_list");
    g_err |= cbor_encoder_create_array(&cb->encoder, &log_list,
                                       CborIndefiniteLength);

    log = NULL;
    while (1) {
        log = log_list_get_next(log);
        if (!log) {
            break;
        }

        if (log->l_log->log_type == LOG_TYPE_STREAM) {
            continue;
        }

        g_err |= cbor_encode_text_stringz(&log_list, log->l_name);
    }

    g_err |= cbor_encoder_close_container(&cb->encoder, &log_list);

    if (g_err) {
        return MGMT_ERR_ENOMEM;
    }
    return (0);
}

/**
 * Newtmgr Log Level list handler
 * @param nmgr json buffer
 * @return 0 on success; non-zero on failure
 */
static int
log_nmgr_level_list(struct mgmt_cbuf *cb)
{
    CborError g_err = CborNoError;
    CborEncoder level_map;
    int level;
    char *str;

    g_err |= cbor_encode_text_stringz(&cb->encoder, "rc");
    g_err |= cbor_encode_int(&cb->encoder, MGMT_ERR_EOK);

    g_err |= cbor_encode_text_stringz(&cb->encoder, "level_map");
    g_err |= cbor_encoder_create_map(&cb->encoder, &level_map,
                                     CborIndefiniteLength);

    level = LOG_LEVEL_DEBUG;
    while (level < LOG_LEVEL_MAX) {
        str = LOG_LEVEL_STR(level);
        if (!strcmp(str, "UNKNOWN")) {
            level++;
            continue;
        }

        g_err |= cbor_encode_text_stringz(&level_map, str);
        g_err |= cbor_encode_uint(&level_map, level);
        level++;
    }

    g_err |= cbor_encoder_close_container(&cb->encoder, &level_map);

    if (g_err) {
        return MGMT_ERR_ENOMEM;
    }
    return (0);
}

/**
 * Newtmgr log clear handler
 * @param nmgr json buffer
 * @return 0 on success; non-zero on failure
 */
static int
log_nmgr_clear(struct mgmt_cbuf *cb)
{
    struct log *log;
    int rc;

    log = NULL;
    while (1) {
        log = log_list_get_next(log);
        if (log == NULL) {
            break;
        }

        if (log->l_log->log_type == LOG_TYPE_STREAM) {
            continue;
        }

        rc = log_flush(log);
        if (rc) {
            return rc;
        }
    }

    rc = mgmt_cbuf_setoerr(cb, 0);
    if (rc != 0) {
        return rc;
    }

    return 0;
}

/**
 * Register nmgr group handlers.
 * @return 0 on success; non-zero on failure
 */
int
log_nmgr_register_group(void)
{
    int rc;

    MGMT_GROUP_SET_HANDLERS(&log_nmgr_group, log_nmgr_group_handlers);
    log_nmgr_group.mg_group_id = MGMT_GROUP_ID_LOGS;

    rc = mgmt_group_register(&log_nmgr_group);
    if (rc) {
        goto err;
    }

    return (0);
err:
    return (rc);
}

#endif
