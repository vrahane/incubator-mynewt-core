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
    /* I2C interface list */
    SLIST_HEAD(, i2c_itf) i_itf_list;
} i2cmgr;

static void i2cmgr_startup_ev_cb(struct os_event *ev);
static void i2cmgr_event_cb(struct os_event *ev);

/*
 * Event for processing I2C manager events (READ/WRITE)
 */
static struct os_event i2cmgr_startup_evt = {
    .ev_cb = i2cmgr_startup_ev_cb,
};

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

/*
 * Post startup event to all I2C interfaces
 */
void
i2cmgr_put_startup_event(void)
{
    struct i2c_itf *ii;

    SLIST_FOREACH(ii, &i2cmgr.i_itf_list, ii_next) {
        ii->ii_startup_ev.ev_arg = ii;
        os_eventq_put(ii->ii_evq, &ii->ii_startup_ev);
    }
}

/**
 * Put job event on the I2C interface eventq
 *
 * @param I2C job ptr
 */
void
i2cmgr_put_evt(struct i2c_job *ij)
{
    struct i2c_itf *ii;

    ii = i2c_get_itf_bynum(ij->ij_itf_num);
    if (!ii) {
        /* No I2C interface found */
        return;
    }

    os_eventq_put(ii->ii_evq, ij->ij_ev);
}

/*
 * event callback for calling job callback
 */
static void
i2cmgr_event_cb(struct os_event *ev)
{
    struct i2c_job *ij;
    int rc;

    ij = (struct i2c_job *)ev->ev_arg;

    ij->ij_cb(ij->ij_arg);

    i2c_job_unlock(ij);
}

/**
 * I2Cmgr timeout handler function
 *
 * @param argument
 */
void
i2cmgr_handle_tmo(void *arg)
{
    struct i2c_job *ij;

    ij = (struct i2c_timer_ctx *)arg;

    i2cmgr_put_evt(ij);
}

/**
 * Initialize i2c_job
 *
 * @param i2c_job to initialize
 *
 * @return 0 on success, non-zero on failure
 */
int
i2cmgr_job_init(struct i2c_job *ij)
{
    os_sem_init(&ij->ij_sem, 0);
    ij->ij_ev.ev_cb  = i2cmgr_event_cb;
    ij->ij_ev.ev_arg = ij;
    os_cputime_timer_init(&ij_timer, i2cmgr_handle_tmo, ij);
}

/**
 * Lock job
 *
 * @param The i2c job to lock
 *
 * @return 0 on success, non-zero on failure
 */
static int
i2c_job_lock(struct i2c_job *ij)
{
    int rc;

    rc = os_sem_pend(&ij->ij_sem, OS_TIMEOUT_NEVER);
    if (rc == 0 || rc == OS_NOT_STARTED) {
        return 0;
    }

    return rc;
}

/**
 * Unlock the job
 *
 * @param The i2c job to unlock
 *
 * @return 0 on success, non-zero on failure
 */
static void
i2c_job_unlock(struct i2c_job *ij)
{
    (void)os_sem_release(&ij->ij_sem);
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
 * Startup event callback for getting the current task
 */
static void
i2cmgr_startup_ev_cb(struct os_event *ev)
{
    struct i2c_itf *ii;

    ii = (struct i2c_itf *)ev->ev_arg;
    ii->ii_task = os_sched_get_current_task();
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
    SYSINIT_PANIC_ASSERT(rc == 0);
#endif
#if MYNEWT_VAL(I2C_1)
    i2cmgr_insert(&i2c_itf1, &prev);
    i2c_itf1.ii_num = 1;
    SYSINIT_PANIC_ASSERT(rc == 0);
#endif
#if MYNEWT_VAL(I2C_2)
    i2cmgr_insert(&i2c_itf2, &prev);
    i2c_itf2.ii_num = 2;
    SYSINIT_PANIC_ASSERT(rc == 0);
#endif
#if MYNEWT_VAL(I2C_3)
    i2cmgr_insert(&i2c_itf3, &prev);
    i2c_itf3.ii_num = 3;
    SYSINIT_PANIC_ASSERT(rc == 0);
#endif


#ifdef MYNEWT_VAL_I2CMGR_I2C_0_EVQ
    i2cmgr_evq_set(&i2c_itf0, MYNEWT_VAL(I2CMGR_I2C_0_EVQ));
#else
    i2cmgr_evq_set(&i2c_itf0, os_eventq_dflt_get());
#endif

#ifdef MYNEWT_VAL_I2CMGR_I2C_1_EVQ
    i2cmgr_evq_set(&i2c_itf1, MYNEWT_VAL(I2CMGR_I2C_1_EVQ));
#else
    i2cmgr_evq_set(&i2c_itf1, os_eventq_dflt_get());
#endif

#ifdef MYNEWT_VAL_I2CMGR_I2C_2_EVQ
    i2cmgr_evq_set(&i2c_itf2, MYNEWT_VAL(I2CMGR_I2C_2_EVQ));
#else
    i2cmgr_evq_set(&i2c_itf2, os_eventq_dflt_get());
#endif

#ifdef MYNEWT_VAL_I2CMGR_I2C_3_EVQ
    i2cmgr_evq_set(&i2c_itf3, MYNEWT_VAL(I2CMGR_I2C_3_EVQ));
#else
    i2cmgr_evq_set(&i2c_itf3, os_eventq_dflt_get());
#endif

#ifdef MYNEWT_VAL_I2CMGR_I2C_4_EVQ
    i2cmgr_evq_set(&i2c_itf4, MYNEWT_VAL(I2CMGR_I2C_4_EVQ));
#else
    i2cmgr_evq_set(&i2c_itf4, os_eventq_dflt_get());
#endif
    i2cmgr_put_startup_event();
}

/* Non-blocking I2C job */
int
i2cmgr_job_noblock(uint8_t i2c_num, struct i2c_job *job, uint32_t delay_us,
                   i2cmgr_data_func_t job_cb, void *job_arg)
{
    struct i2c_itf *ii;

    if (!job_cb || !job_arg) {
        return SYS_EINVAL;
    }

    job->ij_cb  = job_cb;
    job->ij_arg = job_arg;

    if (delay_us) {
        os_cputime_timer_relative(job->ij_timer, delay_us);
    } else {
        i2cmgr_put_evt(job);
    }

    return SYS_EOK;
}

/* Blocking I2C job */
int
i2cmgr_job_exec(uint8_t i2c_num, struct i2c_job *job, uint32_t delay_us,
                i2cmgr_data_func_t job_cb, void *job_arg)
{
    struct i2c_itf *ii;
    int rc;

    if (!job_cb || !job_arg) {
        return SYS_EINVAL;
    }

    job->ij_cb  = job_cb;
    job->ij_arg = job_arg;

    rc = i2c_job_lock(ij);
    if (rc) {
        return;
    }

    if (delay_us) {
        os_cputime_timer_relative(job->ij_timer, delay_us);
    } else {
        i2cmgr_put_evt(job);
    }

    return SYS_EOK;
}

/**
 * Get the current eventq, the system is misconfigured if there is still
 * no parent eventq.
 *
 * @return eventq ptr
 */
struct os_eventq *
i2cmgr_evq_get(uint8_t i2c_num)
{
    struct i2c_itf *ii;

    ii = i2c_get_itf_bynum(i2c_num);
    if (!ii) {
        return NULL;
    }

    return (ii->ii_evq);
}

static void
i2cmgr_evq_set(struct i2c_itf *ii, struct os_eventq *evq)
{
    ii->ii_evq = evq;
}

void
i2cmgr_pkg_init(void)
{
    /* Ensure this function only gets called by sysinit. */
    SYSINIT_ASSERT_ACTIVE();

    i2cmgr_init();
}
