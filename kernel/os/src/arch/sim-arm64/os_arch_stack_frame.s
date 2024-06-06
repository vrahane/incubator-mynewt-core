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

#define sigsetjmp   __sigsetjmp

    .text
    .p2align 4, 0x90    /* align on 16-byte boundary and fill with NOPs */

    .globl os_arch_frame_init
    .type  os_arch_frame_init, %function
    /*
     * void os_arch_frame_init(struct stack_frame *sf)
     */
os_arch_frame_init:
    mov     x1, sp
    mov     sp, x0                /* stack for the task starts from sf */
    sub     sp, sp, #32           /* stack must be aligned by 16 */
    str     x1, [sp, #16]
    str     lr, [sp, #24]         /* Store LR there */
    str     x0, [sp, #8]          /* Store sf pointer to stack */
    add     x0, x0, #8            /* x0 = sf->sf_jb */
    mov     x1, #0
    bl      sigsetjmp
    cbz     x0, end               /* If x0 == 0 then return */
    mov     x1, x0
    ldr     x0, [sp, #8]
    and     sp, sp, #0xfffffffffffffff0
    bl      os_arch_task_start
end:
    ldr     x1, [sp, #24]         /* return sp for the callee */
    ldr     sp, [sp, #16]
    mov     x30, x1
    ret
