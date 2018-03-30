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
 * KIND, either exprcs or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <stdint.h>
#include "hal/hal_i2c.h"
#include "stdio.h"
#include "os/os.h"
#include "os/os_mutex.h"
#include "bsp.h"
#include "i2cmgr/i2cmgr.h"
#include "defs/error.h"
#include "assert.h"
#include "sysinit/sysinit.h"

/* I2C Mgr Task */
#define I2CMGR_TASK_PRIO (MYNEWT_VAL(I2CMGR_TASK_PRIO))
#define I2CMGR_STACK_SIZE OS_STACK_ALIGN(MYNEWT_VAL(I2CMGR_STACK_SIZE))

static struct os_task i2cmgr_task;

static os_membuf_t i2c_job_mem[
    OS_MEMPOOL_SIZE(MYNEWT_VAL(I2C_JOB_MPOOL_MAX_NUM),
                    sizeof (struct i2c_job))
];

static struct os_mempool i2c_job_pool;

static os_membuf_t i2c_job_op_mem[
    OS_MEMPOOL_SIZE(MYNEWT_VAL(I2C_JOB_OP_MPOOL_MAX_NUM),
                    sizeof (struct i2c_job_op))
];

static struct os_mempool i2c_job_op_pool;

#define I2C(n) I2C_##n

#if MYNEWT_VAL(I2C_3)
#define MAX_I2C 4
struct i2c_itf i2c_itf3;
#elif MYNEWT_VAL(I2C_2)
struct i2c_itf i2c_itf2;
#define MAX_I2C 3
#elif MYNEWT_VAL(I2C_1)
struct i2c_itf i2c_itf1;
#define MAX_I2C 2
#elif MYNEWT_VAL(I2C_0)
struct i2c_itf i2c_itf0;
#define MAX_I2C 1
#endif

/*
 * I2C manager which consists of the eventqand the HEAD of the interface list
 */
struct i2cmgr {
    /* I2C job eventq */
    struct os_eventq i_evq;
    /* I2C interface list */
    SLIST_HEAD(, i2c_itf) i_itf_list;
} i2cmgr;

static void i2cmgr_ev_cb(struct os_event *ev);

/*
 * Event for processing I2C manager events (READ/WRITE)
 */
static struct os_event i2cmgr_evt = {
    .ev_cb = i2cmgr_ev_cb,
};

/*
 * Allocate memory block for an I2C job
 */
static struct i2c_job *
i2c_job_alloc(void)
{
    struct i2c_job *ij;

    ij = os_memblock_get(&i2c_job_pool);
    if (ij) {
        memset(ij, 0, sizeof(*ij));
    }

    return ij;
}

/*
 * Allocate memory block for an I2C job op
 */
static struct i2c_job_op *
i2c_job_op_alloc(void)
{
    struct i2c_job_op *ijo;

    ijo = os_memblock_get(&i2c_job_op_pool);
    if (ijo) {
        memset(ijo, 0, sizeof(*ijo));
    }

    return ijo;
}

/*
 * Free memory block for an I2C job back to the pool
 */
static int
i2c_job_free(struct i2c_job *ij)
{
     return os_memblock_put(&i2c_job_pool, ij);
}

/*
 * Free memory block for an I2C job op back to the pool
 */
static int
i2c_job_op_free(struct i2c_job_op *ijo)
{
    return os_memblock_put(&i2c_job_op_pool, ijo);
}

/*
 * Insert I2C interface in the interface list
 */
void
i2cmgr_insert(struct i2c_itf *ii, struct i2c_itf **prev)
{
    if (*prev == NULL) {
        SLIST_INSERT_HEAD(&i2cmgr.i_itf_list, ii, ii_next);
    } else {
        SLIST_INSERT_AFTER(*prev, ii, ii_next);
    }

    *prev = ii;
}

/*
 * Remove I2C interface from the interface list
 */
void
i2cmgr_remove(struct i2c_itf *ii)
{
    SLIST_REMOVE(&i2cmgr.i_itf_list, ii, i2c_itf, ii_next);
}

/**
 * Lock job list
 *
 * @param The i2c interface to lock the job list for
 *
 * @return 0 on success, non-zero on failure
 */
static int
i2c_jobq_lock(struct i2c_itf *ii)
{
    int rc;

    rc = os_mutex_pend(&ii->ii_jobq_lock, OS_TIMEOUT_NEVER);
    if (rc == 0 || rc == OS_NOT_STARTED) {
        return 0;
    }

    return rc;
}

/**
 * Unlock the job list
 *
 * @param The i2c interface to unlock the job list for
 *
 * @return 0 on success, non-zero on failure
 */
static void
i2c_jobq_unlock(struct i2c_itf *ii)
{
    (void)os_mutex_release(&ii->ii_jobq_lock);
}

/**
 * Get I2C interface by number
 *
 * @param I2C interface number
 *
 * @return I2C interface ptr matching the number, NULL if not found
 */
static struct i2c_itf *
i2c_get_itf_bynum(uint8_t i2c_num)
{
    struct i2c_itf *cursor;

    SLIST_FOREACH(cursor, &i2cmgr.i_itf_list, ii_next) {
        if (cursor->ii_num == i2c_num) {
            return cursor;
        }
    }

    return NULL;
}

/*
 * Event callback for getting a job and processing it
 */
static void
i2cmgr_ev_cb(struct os_event *ev)
{
    struct i2c_itf *ii;
    struct i2c_job *ij;
    int rc;

    SLIST_FOREACH(ii, &i2cmgr.i_itf_list, ii_next) {

        if (ii) {
            ij = i2c_get_high_prio_job(ii);
            if (!ij) {
                return;
            }

            rc = i2c_process_job(ij, ii->ii_num);
            if (rc) {
                return;
            }

            rc = i2c_remove_job(ii->ii_num, ij);
            if (rc) {
                /* XXX Log failure */
                return;
            }
        }
    }
}

/*
 * I2C manager task handler
 */
static void
i2cmgr_handler(void *arg)
{
    struct os_task *t;

    while (1) {
        t = os_sched_get_current_task();
        assert(t->t_func == i2cmgr_handler);

        os_eventq_run(&i2cmgr.i_evq);
    }
}

/*
 * Initialize I2C manager
 */
void
i2cmgr_init(void)
{
    struct i2c_itf *prev;
    os_stack_t *i2cmgr_stack;
    int rc;

    prev = NULL;
    rc = 0;

#if MYNEWT_VAL(I2C_0)
    i2cmgr_insert(&i2c_itf0, &prev);
    i2c_itf0.ii_num = 0;
    rc = os_mutex_init(&i2c_itf0.ii_jobq_lock);
    SYSINIT_PANIC_ASSERT(rc == 0);
#endif
#if MYNEWT_VAL(I2C_1)
    i2cmgr_insert(&i2c_itf1, &prev);
    i2c_itf1.ii_num = 1;
    rc = os_mutex_init(&i2c_itf1.ii_jobq_lock);
    SYSINIT_PANIC_ASSERT(rc == 0);
#endif
#if MYNEWT_VAL(I2C_2)
    i2cmgr_insert(&i2c_itf2, &prev);
    i2c_itf2.ii_num = 2;
    rc = os_mutex_init(&i2c_itf2.ii_jobq_lock);
    SYSINIT_PANIC_ASSERT(rc == 0);
#endif
#if MYNEWT_VAL(I2C_3)
    i2cmgr_insert(&i2c_itf3, &prev);
    i2c_itf3.ii_num = 3;
    rc = os_mutex_init(&i2c_itf3.ii_jobq_lock);
    SYSINIT_PANIC_ASSERT(rc == 0);
#endif

    /* allocate I2C manager stack */
    i2cmgr_stack = malloc(sizeof(os_stack_t) * I2CMGR_STACK_SIZE);
    assert(i2cmgr_stack);

    /* Initialize eventq for i2cmgr task */
    os_eventq_init(&i2cmgr.i_evq);
}

/**
 * Puts an I2C event on the i2cmgr evq
 *
 * @param I2C mgr event context
 */
void
i2cmgr_put_evt(void *arg)
{
    i2cmgr_evt.ev_arg = arg;
    os_eventq_put(&i2cmgr.i_evq, &i2cmgr_evt);
}

void
i2cmgr_pkg_init(void)
{
    /* Ensure this function only gets called by sysinit. */
    SYSINIT_ASSERT_ACTIVE();

    i2cmgr_init();
}
