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

#include <assert.h>
#include <string.h>

#include "os/mynewt.h"

#include <mgmt/mgmt.h>
#include <smp_os/smp_os.h>

#include <cborattr/cborattr.h>
#include <tinycbor/cbor.h>
#include <tinycbor/cbor_mbuf_reader.h>
#include <oic/oc_api.h>

struct omgr_ctxt {
    struct mgmt_ctxt ob_mc;
    struct cbor_mbuf_reader ob_reader;
};

struct omgr_state {
    struct omgr_ctxt os_ctxt;		/* CBOR buffer for MGMT task */
};

static struct omgr_state omgr_state;

static int
omgr_oic_read_hdr(struct CborValue *cv, struct mgmt_hdr *out_hdr)
{
    size_t hlen;
    int rc;

    struct cbor_attr_t attrs[] = {
        [0] = {
            .attribute = "_h",
            .type = CborAttrByteStringType,
            .addr.bytestring.data = (void *)out_hdr,
            .addr.bytestring.len = &hlen,
            .nodefault = 1,
            .len = sizeof *out_hdr,
        },
        [1] = { 0 }
    };

    rc = cbor_read_object(cv, attrs);
    if (rc != 0 || hlen != sizeof *out_hdr) {
        return MGMT_ERR_EINVAL;
    }

    out_hdr->nh_len = ntohs(out_hdr->nh_len);
    out_hdr->nh_group = ntohs(out_hdr->nh_group);

    return 0;
}

static int
omgr_encode_mgmt_hdr(struct CborEncoder *enc, struct mgmt_hdr hdr)
{
    int rc;

    rc = cbor_encode_text_string(enc, "_h", 2);
    if (rc != 0) {
        return MGMT_ERR_ENOMEM;
    }

    hdr.nh_len = htons(hdr.nh_len);
    hdr.nh_group = htons(hdr.nh_group);

    /* Encode the MGMT header in the response. */
    rc = cbor_encode_byte_string(enc, (void *)&hdr, sizeof hdr);
    if (rc != 0) {
        return MGMT_ERR_ENOMEM;
    }

    return 0;
}

static int
omgr_send_err_rsp(struct CborEncoder *enc, const struct mgmt_hdr *hdr,
                  int mgmt_status)
{
    int rc;

    rc = omgr_encode_mgmt_hdr(enc, *hdr);
    if (rc != 0) {
        return rc;
    }

    rc = cbor_encode_text_stringz(enc, "rc");
    if (rc != 0) {
        return MGMT_ERR_ENOMEM;
    }

    rc = cbor_encode_int(enc, mgmt_status);
    if (rc != 0) {
        return MGMT_ERR_ENOMEM;
    }

    return 0;
}


int
omgr_extract_req_hdr(oc_request_t *req, struct mgmt_hdr *out_hdr)
{
    struct omgr_state *o = &omgr_state;
    uint16_t data_off;
    struct os_mbuf *m;
    int rc;

    coap_get_payload(req->packet, &m, &data_off);

    cbor_mbuf_reader_init(&o->os_ctxt.ob_reader, m, data_off);
    cbor_parser_init(&o->os_ctxt.ob_reader.r, 0, &o->os_ctxt.ob_mc.parser,
                     &o->os_ctxt.ob_mc.it);

    rc = omgr_oic_read_hdr(&o->os_ctxt.ob_mc.it, out_hdr);
    if (rc != 0) {
        return MGMT_ERR_EINVAL;
    }

    return 0;
}

int
omgr_oic_process_put(oc_request_t *req, oc_interface_mask_t mask)
{
    struct omgr_state *o = &omgr_state;
    const struct mgmt_handler *handler;
    uint16_t data_off;
    struct os_mbuf *m;
    int rc = 0;
    struct mgmt_hdr hdr;
    int rsp_hdr_filled = 0;

    coap_get_payload(req->packet, &m, &data_off);

    cbor_mbuf_reader_init(&o->os_ctxt.ob_reader, m, data_off);
    cbor_parser_init(&o->os_ctxt.ob_reader.r, 0, &o->os_ctxt.ob_mc.parser,
                     &o->os_ctxt.ob_mc.it);

    rc = omgr_oic_read_hdr(&o->os_ctxt.ob_mc.it, &hdr);
    if (rc != 0) {
        rc = MGMT_ERR_EINVAL;
        goto done;
    }

    /* Convert request header to response header to be sent. */
    switch (hdr.nh_op) {
    case MGMT_OP_READ:
        hdr.nh_op = MGMT_OP_READ_RSP;
        break;

    case MGMT_OP_WRITE:
        hdr.nh_op = MGMT_OP_WRITE_RSP;
        break;

    default:
        goto done;
    }
    rsp_hdr_filled = 1;

    /* Begin root map in response. */
    cbor_encoder_create_map(&g_encoder, &o->os_ctxt.ob_mc.encoder,
                            CborIndefiniteLength);

    handler = mgmt_find_handler(hdr.nh_group, hdr.nh_id);
    if (handler == NULL) {
        rc = MGMT_ERR_ENOENT;
        goto done;
    }

    cbor_mbuf_reader_init(&o->os_ctxt.ob_reader, m, data_off);
    cbor_parser_init(&o->os_ctxt.ob_reader.r, 0, &o->os_ctxt.ob_mc.parser,
                     &o->os_ctxt.ob_mc.it);

    switch (mask) {
    case OC_IF_BASELINE:
        oc_process_baseline_interface(req->resource);
        /* Fallthrough */

    case OC_IF_RW:
        switch (hdr.nh_op) {
        case MGMT_OP_READ_RSP:
            if (handler->mh_read == NULL) {
                rc = MGMT_ERR_ENOENT;
            } else {
                rc = handler->mh_read(&o->os_ctxt.ob_mc);
            }
            break;

        case MGMT_OP_WRITE_RSP:
            if (handler->mh_write == NULL) {
                rc = MGMT_ERR_ENOENT;
            } else {
                rc = handler->mh_write(&o->os_ctxt.ob_mc);
            }
            break;

        default:
            rc = MGMT_ERR_EINVAL;
            break;
        }
        if (rc != 0) {
            goto done;
        }

        /* Encode the MGMT header in the response. */
        rc = omgr_encode_mgmt_hdr(&o->os_ctxt.ob_mc.encoder, hdr);
        if (rc != 0) {
            rc = MGMT_ERR_ENOMEM;
            goto done;
        }
        break;

    default:
        break;
    }

    rc = 0;

done:
    if (rc != 0) {
        if (rsp_hdr_filled) {
            rc = omgr_send_err_rsp(&g_encoder, &hdr, rc);
        }
        switch (rc) {
        case 0:
            break;

        case MGMT_ERR_ENOMEM:
            rc = OC_STATUS_INTERNAL_SERVER_ERROR;
            break;

        default:
            rc = OC_STATUS_BAD_REQUEST;
            break;
        }
    }

    cbor_encoder_close_container(&g_encoder, &o->os_ctxt.ob_mc.encoder);
    oc_send_response(req, rc);

    return rc;
}

static void
omgr_oic_put(oc_request_t *req, oc_interface_mask_t mask)
{
    omgr_oic_process_put(req, mask);
}

int
oicmgr_init(void)
{
    uint8_t mode;
    oc_resource_t *res = NULL;

    /* Ensure this function only gets called by sysinit. */
    SYSINIT_ASSERT_ACTIVE();

    /*
     * net/oic must be initialized before now.
     */
    res = oc_new_resource("/omgr", 1, 0);
    oc_resource_bind_resource_type(res, "x.mynewt.nmgr");
    mode = OC_IF_RW;
    oc_resource_bind_resource_interface(res, mode);
    oc_resource_set_default_interface(res, mode);
    oc_resource_set_discoverable(res);
    oc_resource_set_request_handler(res, OC_PUT, omgr_oic_put);
    res->properties |= MYNEWT_VAL(OICMGR_TRANS_SECURITY);
    oc_add_resource(res);

    return (0);
}
