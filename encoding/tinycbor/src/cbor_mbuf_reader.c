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

#include <tinycbor/cbor_mbuf_reader.h>
#include <tinycbor/compilersupport_p.h>
#include <os/os_mbuf.h>

static bool
cbor_mbuf_can_read_bytes(void *token, size_t len);
static void *
cbor_mbuf_read_bytes(void *token, void *dst, size_t offset, size_t len);
static void
cbor_mbuf_advance_bytes(void *token, size_t len);
static CborError
cbor_mbuf_transfer_string(void *token, const void **userptr, size_t offset,
                          size_t len);

struct CborParserOperations cbor_mbuf_parser_ops = {
    .can_read_bytes = cbor_mbuf_can_read_bytes,
    .read_bytes = cbor_mbuf_read_bytes,
    .advance_bytes = cbor_mbuf_advance_bytes,
    .transfer_string = cbor_mbuf_transfer_string
};

static bool
cbor_mbuf_can_read_bytes(void *token, size_t len)
{
    struct cbor_mbuf_reader *cb = (struct cbor_mbuf_reader *)token;

    return (OS_MBUF_PKTLEN(cb->m) - cb->init_off) >= (int)len;
}

static void *
cbor_mbuf_read_bytes(void *token, void *dst, size_t offset, size_t len)
{
    struct cbor_mbuf_reader *cb = (struct cbor_mbuf_reader *)token;
    os_mbuf_copydata(cb->m, offset + cb->init_off, len, dst);
    if (len == sizeof(uint8_t)) {
        goto done;
    } else if (len == sizeof(uint16_t)) {
        *(uint16_t *)dst = cbor_ntohs(*(uint16_t *)dst);
        goto done;
    } else if (len == sizeof(uint32_t)) {
        *(uint32_t *)dst = cbor_ntohl(*(uint32_t *)dst);
        goto done;
    } else if (len == sizeof(uint64_t)) {
        *(uint64_t *)dst = cbor_ntohll(*(uint64_t *)dst);
        goto done;
    }

done:
    return dst;
}

static void
cbor_mbuf_advance_bytes(void *token, size_t len)
{
    struct cbor_mbuf_reader *cb = (struct cbor_mbuf_reader *)token;

    cb->init_off += len;
}

static CborError
cbor_mbuf_transfer_string(void *token, const void **userptr, size_t offset,
                          size_t len)
{
    struct cbor_mbuf_reader *cb = (struct cbor_mbuf_reader *)token;
    char ch;
    int rc;
    size_t prev_ioff;

    if (OS_MBUF_PKTLEN(cb->m) - cb->init_off < (int)(len + offset)) {
        return CborErrorUnexpectedEOF;
    }

    cb->init_off += (int)offset;

    rc = os_mbuf_copydata(cb->m, cb->init_off, 1, &ch);
    prev_ioff = cb->init_off;
    while(!rc && ch != '\0') {
        rc = os_mbuf_copydata(cb->m, prev_ioff, 1, &ch);
        prev_ioff++;
    }

    if (!rc && ch == '\0') {
        cb->init_off = prev_ioff - 1;
        *userptr = cb->m;
    }

    return !rc ? CborNoError : CborErrorOutOfMemory;
}

/*
void
cbor_mbuf_reader_init(struct cbor_mbuf_reader *cb, struct os_mbuf *m,
                      int initial_offset)
{
    struct os_mbuf_pkthdr *hdr;

    assert(OS_MBUF_IS_PKTHDR(m));
    hdr = OS_MBUF_PKTHDR(m);
    cb->m = m;
    cb->init_off = initial_offset;
    cb->r.message_size = hdr->omp_len - initial_offset;
}
*/
