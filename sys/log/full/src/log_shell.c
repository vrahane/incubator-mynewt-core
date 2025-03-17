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

#include "os/mynewt.h"

/* This whole file is conditionally compiled based on whether the
 * log package is configured to use the shell (MYNEWT_VAL(LOG_CLI)).
 */

#if MYNEWT_VAL(LOG_CLI)

#include <stdio.h>
#include <string.h>
#include <parse/parse.h>
#include <ctype.h>

#include "cbmem/cbmem.h"
#include "log/log.h"
#include "shell/shell.h"
#include "console/console.h"
#include "base64/hex.h"
#include "tinycbor/cbor.h"
#include "tinycbor/compilersupport_p.h"
#include "log_cbor_reader/log_cbor_reader.h"

void log_console_print_hdr(const struct log_entry_hdr *hdr);

static uint32_t shell_log_count;


struct walk_arg {
    /* Number of entries to skip */
    uint32_t skip;
    /* Number of entries to process */
    uint32_t count_limit;
    /* Entry number */
    uint32_t count;
    /* Entry index */
    uint32_t idx;
};

static int
shell_log_count_entry(struct log *log, struct log_offset *log_offset,
                      const struct log_entry_hdr *ueh, const void *dptr, uint16_t len)
{
    struct walk_arg *arg = (struct walk_arg *)log_offset->lo_arg;

    shell_log_count++;
    if (arg) {
        arg->count++;
        if ((arg->count_limit > 0) && (arg->count >= arg->count_limit)) {
            return 1;
        }
    }

    return 0;
}

static int
shell_log_dump_entry(struct log *log, struct log_offset *log_offset,
                     const struct log_entry_hdr *ueh, const void *dptr, uint16_t len)
{
    char data[128 + 1];
    int dlen;
    int rc = 0;
    struct CborParser cbor_parser;
    struct CborValue cbor_value;
    struct log_cbor_reader cbor_reader;
    char tmp[32 + 1];
    int off;
    int blksz;
    bool read_data = ueh->ue_etype != LOG_ETYPE_CBOR;
    bool read_hash = ueh->ue_flags & LOG_FLAGS_IMG_HASH;

    dlen = min(len, 128);

    if (read_data) {
        rc = log_read_body(log, dptr, data, 0, dlen);
        if (rc < 0) {
            return rc;
        }
        data[rc] = 0;
    }

    if (read_hash) {
        console_printf("[ih=0x%x%x%x%x]", ueh->ue_imghash[0], ueh->ue_imghash[1],
                       ueh->ue_imghash[2], ueh->ue_imghash[3]);
    }
    console_printf(" [%llu] ", ueh->ue_ts);
#if MYNEWT_VAL(LOG_SHELL_SHOW_INDEX)
    console_printf(" [ix=%lu] ", ueh->ue_index);
#endif

    switch (ueh->ue_etype) {
    case LOG_ETYPE_STRING:
        console_write(data, strlen(data));
        break;
    case LOG_ETYPE_CBOR:
        log_cbor_reader_init(&cbor_reader, log, dptr, len);
        cbor_parser_init(&cbor_reader.r, 0, &cbor_parser, &cbor_value);
        cbor_value_to_pretty(stdout, &cbor_value);
        break;
    default:
        for (off = 0; off < rc; off += blksz) {
            blksz = dlen - off;
            if (blksz > sizeof(tmp) >> 1) {
                blksz = sizeof(tmp) >> 1;
            }
            hex_format(&data[off], blksz, tmp, sizeof(tmp));
            console_write(tmp, strlen(tmp));
        }
        if (rc < len) {
            console_write("...", 3);
        }
    }

    console_write("\n", 1);
    return 0;
}

int
shell_log_dump_cmd(int argc, char **argv)
{
    struct log *log;
    struct log_offset log_offset;
    bool list_only = false;
    char *log_name = NULL;
    uint32_t log_last_index = 0;
    uint32_t log_limit = 0;
    bool stream;
    bool partial_match = false;
    bool clear_log;
    bool dump_logs = true;
    bool dump_bmarks = false;
    uint32_t bmarks_size = 0;
    struct log_fcb_bmark *bmarks = NULL;
    struct walk_arg arg = {};
    int i;
    int rc = 0;
    int start = -1;
    int end = -1;

    clear_log = false;
    (void)dump_bmarks;
    (void)bmarks;
    (void)bmarks_size;
    (void)start;
    (void)end;

    for (i = 1; i < argc; ++i) {
        if (0 == strcmp(argv[i], "-l")) {
            list_only = true;
            break;
        }
        if (0 == strcmp(argv[i], "-n")) {
            if (i + 1 < argc) {
                arg.count_limit = parse_ll_bounds(argv[i + 1], 1, 1000000, &rc);
                if (rc) {
                    arg.count_limit = 1;
                }
                log_offset.lo_arg = &arg;
            }
            ++i;
            continue;
        }
        if (0 == strcmp(argv[i], "-s")) {
            if (i + 1 < argc) {
                arg.skip = parse_ll_bounds(argv[i + 1], 0, 1000000, &rc);
                if (rc) {
                    arg.skip = 0;
                }
                log_offset.lo_arg = &arg;
            }
            ++i;
            continue;
        }
        if (0 == strcmp(argv[i], "-t")) {
            dump_logs = false;
            continue;
        }
        if (0 == strcmp(argv[i], "-b")) {
            dump_logs = false;
            dump_bmarks = true;
            continue;
        }
        if (0 == strcmp(argv[i], "-i")) {
            if (i + 1 < argc) {
                arg.idx = parse_ll_bounds(argv[i + 1], 0, UINT32_MAX, &rc);
                if (rc) {
                    arg.idx = 0;
                }
                log_offset.lo_arg = &arg;
            }
            ++i;
            continue;
        }

        /* the -c option is to clear a log (or logs). */
        if (!strcmp(argv[i], "-c")) {
            clear_log = true;
        } else if (isdigit((unsigned char)argv[i][0])) {
            log_limit = parse_ll_bounds(argv[i], 1, 1000000, &rc);
            if (clear_log) {
                goto err;
            }
        } else {
            log_name = argv[i];
            if ('*' == log_name[strlen(log_name) - 1]) {
                partial_match = true;
                log_name[strlen(log_name) - 1] = '\0';
            }
        }
    }

    log = NULL;
    while (1) {
        log = log_list_get_next(log);
        if (log == NULL) {
            break;
        }

        stream = log->l_log->log_type == LOG_TYPE_STREAM;

        if (list_only) {
            console_printf("%s%s\n", log->l_name,
                           stream ? " (stream)" : "");
            continue;
        }

        if (stream ||
            (log_name != NULL && ((partial_match && (log->l_name != strstr(log->l_name, log_name))) ||
                                  (!partial_match && 0 != strcmp(log->l_name, log_name))))) {
            continue;
        }

#if MYNEWT_VAL(LOG_FCB_BOOKMARKS)
        if (dump_bmarks) {
            bmarks = log_fcb_get_bmarks(log, &bmarks_size);
            for (i = 0; i < bmarks_size; i++) {
#if MYNEWT_VAL(LOG_FCB)
                if (!bmarks[i].lfb_entry.fe_area) {
                    if (start == -1) {
                        start = i;
                    }
                    end = i;
                    continue;
                }
                if (start != -1) {
                    console_printf("bookmarks unused: %d to %d\n", start, end);
                    start = -1;
                    end = -1;
                }
                console_printf("%u: index:%lu fa_off:%x fe_elem_off:%lx\n", i,
                               bmarks[i].lfb_index,
                               (uintptr_t)bmarks[i].lfb_entry.fe_area->fa_off,
                               bmarks[i].lfb_entry.fe_elem_off);
#else
                if (!bmarks[i].lfb_entry.fe_range) {
                    if (start == -1) {
                        start = i;
                    }
                    end = i;
                    continue;
                }
                if (start != -1) {
                    console_printf("bookmarks unused: %d to %d\n", start, end);
                    start = -1;
                    end = -1;
                }
                console_printf("%u: index:%lu fa_off:%x fe_sector:%x fe_data_off:%lx\n", i,
                               bmarks[i].lfb_index,
                               (uintptr_t)bmarks[i].lfb_entry.fe_range->fsr_flash_area.fa_off,
                               (uintptr_t)bmarks[i].lfb_entry.fe_sector,
                               bmarks[i].lfb_entry.fe_data_off);
#endif
            }

            if (start != -1) {
                console_printf("bookmarks unused: %d to %d\n", start, end);
                start = -1;
                end = -1;
            }
            goto err;
        }
#endif

        if (clear_log) {
            console_printf("Clearing log %s\n", log->l_name);
            rc = log_flush(log);
            if (rc != 0) {
                goto err;
            }
        } else {
            console_printf("Dumping log %s\n", log->l_name);

            log_offset.lo_arg = NULL;
            log_offset.lo_ts = 0;
            log_last_index = log_get_last_index(log);
            if (log_limit == 0 || log_last_index < log_limit) {
                log_offset.lo_index = 0;
            } else {
                log_offset.lo_index = log_last_index - log_limit;
            }
            log_offset.lo_data_len = 0;

            if (dump_logs) {
                arg.count = 0;
                log_offset.lo_index = arg.idx;
                rc = log_walk_body(log, shell_log_dump_entry, &log_offset);
            } else if (!dump_bmarks) {
                /* Measure time for log_walk */
                shell_log_count = 0;
                os_time_t start = os_time_get();
                log_offset.lo_index = arg.idx;
                rc = log_walk_body(log, shell_log_count_entry, &log_offset);
                os_time_t end = os_time_get();
                console_printf("Log %s %d entries walked in %d ms\n", log->l_name,
                               (int)shell_log_count, (int)os_time_ticks_to_ms32(end - start));
            }
            if (rc != 0) {
                goto err;
            }
        }
    }

    return (0);
err:
    return (rc);
}


#if MYNEWT_VAL(LOG_STORAGE_INFO)
int
shell_log_storage_cmd(int argc, char **argv)
{
    struct log *log;
    struct log_storage_info info;

    log = NULL;
    while (1) {
        log = log_list_get_next(log);
        if (log == NULL) {
            break;
        }

        if (log->l_log->log_type == LOG_TYPE_STREAM) {
            continue;
        }

        if (log_storage_info(log, &info)) {
            console_printf("Storage info not supported for %s\n", log->l_name);
        } else {
            console_printf("%s: %d of %d used\n", log->l_name,
                           (unsigned)info.used, (unsigned)info.size);
#if MYNEWT_VAL(LOG_STORAGE_WATERMARK)
            console_printf("%s: %d of %d used by unread entries\n", log->l_name,
                           (unsigned)info.used_unread, (unsigned)info.size);
#endif
        }
    }

    return (0);
}
#endif

#endif
