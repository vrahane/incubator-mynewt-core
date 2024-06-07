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

#if defined MN_LINUX
#define sigsetjmp   __sigsetjmp
#define CNAME(x)    x
#elif defined MN_OSX
#define sigsetjmp   sigsetjmp
#define CNAME(x)    _ ## x
#elif defined MN_FreeBSD
#define sigsetjmp   sigsetjmp
#define CNAME(x)    x
#else
#error "unsupported platform"
#endif

    .text
    .p2align 4, 0x90    // Align on 16-byte boundary and fill with NOPs

    .globl CNAME(os_arch_frame_init)
    .globl _os_arch_frame_init

/*
 * void os_arch_frame_init(struct stack_frame *sf)
 */
CNAME(os_arch_frame_init):
    stp x29, x30, [sp, #-16]!       // Save frame pointer and link register
    mov x29, sp                     // Set frame pointer
    stp x19, x20, [sp, #-16]!       // Save x19 and x20 (callee-saved registers)

    /*
     * At this point we are executing on the main() stack:
     * ----------------
     * stack_frame ptr      0x10(sp)
     * ----------------
     * return address       0x8(sp)
     * ----------------
     * saved x29            0x0(sp)
     * ----------------
     */
    ldr x19, [sp, #16]              // x19 = 'sf'
    str x29, [x19]                  // sf->mainsp = x29

    /*
     * Switch the stack so the stack pointer stored in 'sf->sf_jb' points
     * to the task stack. This is slightly complicated because ARM64 requires
     * the stack pointer to be 16-byte aligned.
     *
     * ----------------
     * sf (other fields)
     * ----------------
     * sf (sf_jb)           0x8(x19)
     * ----------------
     * sf (sf_mainsp)       0x0(x19)
     * ----------------
     * alignment padding    variable (0 to 12 bytes)
     * ----------------
     * savemask (0)         0x8(sp)
     * ----------------
     * pointer to sf_jb     0x0(sp)
     * ----------------
     */
    mov x20, sp                     // Save current stack pointer in x20
    sub sp, sp, #16                 // Make room for sigsetjmp() arguments
    mov x9, sp                      // Copy sp to a temporary register
    and x9, x9, #0xfffffff0         // Align x9 to a 16-byte boundary
    mov sp, x9                      // Move the aligned value back to sp
    add x0, x19, #8                 // x0 = &sf->sf_jb
    mov x1, #0                      // x1 = 0 (savemask argument)
    bl CNAME(sigsetjmp)             // sigsetjmp(sf->sf_jb, 0)
    cbnz x0, 1f                     // If return value != 0, jump to label 1
    ldr x29, [x19]                  // Restore main() stack pointer
    mov sp, x29                     // Switch back to the main() stack
    ldp x19, x20, [sp], #16         // Restore x19 and x20
    ldp x29, x30, [sp], #16         // Restore frame pointer and link register
    ret                             // Return to os_arch_task_stack_init()

1:
    adr x2, 2f                      // x2 = address of label 2
    stp x2, xzr, [sp, #-16]!        // Push return address and frame pointer
    mov x29, sp                     // Set frame pointer
    stp x0, x19, [sp, #-16]!        // Push rc and sf
    bl CNAME(os_arch_task_start)    // os_arch_task_start(sf, rc)
    // Never returns

2:
    nop
