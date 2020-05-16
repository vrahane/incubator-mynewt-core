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

#include <arm_cmse.h>
#include "spm_nsc_functions.h"
#include "secure_port_macros.h"

/**
 * @brief Counter returned from spm_nsc_function.
 */
static uint32_t secure_counter = 0;

/**
 * @brief typedef for non-secure callback.
 */
typedef void ( *nonsecure_cb_t ) ( void ) __attribute__( ( cmse_nonsecure_call ) );

__attribute__((cmse_nonsecure_entry)) __attribute__((used)) uint32_t
spm_nsc_function(spm_nsc_func_cb_t px_cb)
{
    nonsecure_cb_t px_nonsecure_cb;

    /* Return function pointer with cleared LSB. */
    px_non_secure_cb = (non_secure_cb_t) cmse_nsfptr_create(px_cb);

    /* Invoke the supplied callback. */
    px_nonsecure_cb();

    /* Increment the secure side counter. */
    secure_counter++;

    /* Return the secure side counter. */
    return secure_counter;
}
