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

#ifndef __SPM_NSC_FUNCTIONS_H__
#define __SPM_NSC_FUNCTIONS_H__

#include <stdint.h>

/**
 * @brief Callback function pointer definition.
 */
typedef void (*spm_nsc_func_cb_t) (void);

/**
 * @brief Invokes the supplied callback which is on the non-secure side.
 *
 * Returns a number which is one more than the value returned in previous
 * invocation of this function. Initial invocation returns 1.
 *
 * @param px_cb The callback to invoke.
 *
 * @return A number which is one more than the value returned in previous
 * invocation of this function.
 */
uint32_t spm_nsc_function(spm_nsc_func_cb_t px_cb);

#endif /* __SPM_NSC_FUNCTIONS_H__ */
