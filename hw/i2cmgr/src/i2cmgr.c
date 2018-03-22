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

/*
 * Get the highest priority job in the job list which is the HEAD of the list
 * in this case
 */
static struct i2c_job *
i2c_get_high_prio_job(struct i2c_itf *ii)
{
    return SLIST_FIRST(&ii->ii_job_list);
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
 * Insert a job in the job list
 *
 * @param I2C interface number
 * @param I2C job
 *
 * @return 0 on success, non-zero on failure
 */
int
i2c_insert_job(uint8_t i2c_num, struct i2c_job *ij)
{
    struct i2c_job *cursor, *prev;
    struct i2c_itf *ii;
    int rc;

    /* Get the I2C interface with the I2C number */
    ii = i2c_get_itf_bynum(i2c_num);
    if (!ii) {
        rc = SYS_EINVAL;
        goto err;
    }

    prev = NULL;

    rc = i2c_jobq_lock(ii);
    if (rc) {
        goto err;
    }

    SLIST_FOREACH(cursor, &ii->ii_job_list, ij_next) {
        if (cursor->ij_prio < ij->ij_prio) {
            break;
        }
        prev = cursor;
    }

    if (prev == NULL) {
        SLIST_INSERT_HEAD(&ii->ii_job_list, ij, ij_next);
    } else {
        SLIST_INSERT_AFTER(cursor, ij, ij_next);
    }

    i2c_jobq_unlock(ii);

    i2cmgr_put_evt(NULL);

    return 0;
err:
    return rc;
}

/*
 * Remove a job from the job list
 *
 * @param I2C interface number
 * @param I2C job
 *
 * @return 0 on success, non-zero on failure
 */
int
i2c_remove_job(uint8_t i2c_num, struct i2c_job *ij)
{
    struct i2c_itf *ii;
    int rc;

    /* Get the I2C interface with the I2C number */
    ii = i2c_get_itf_bynum(i2c_num);
    if (!ii) {
        rc = SYS_EINVAL;
        goto err;
    }

    rc = i2c_jobq_lock(ii);
    if (rc) {
        goto err;
    }

    SLIST_REMOVE(&ii->ii_job_list, ij, i2c_job, ij_next);

    /* Free the job back to the pool */
    rc = i2c_job_free(ij);

    i2c_jobq_unlock(ii);

err:
    return rc;
}

/*
 * Process a job op, for I2C, there are just two ops READ and WRITE that are
 * available to an application
 */
static int
i2c_process_job_op(struct i2c_job_op *ijo, uint8_t ii_num)
{
    int rc;
    struct i2c_itf *ii;
    int64_t currtime;

    rc = SYS_EOK;

    if (ijo->ijo_delay) {
        currtime = os_get_uptime_usec();
        /* If we are not passed the delay */
        if ((currtime - ijo->ijo_prev_uptime) < ijo->ijo_delay) {
            /* job not completed, will have to revisit */
            rc = SYS_EAGAIN;
        }

        ii = i2c_get_itf_bynum(ii_num);

        /* update previous uptime */
        ijo->ijo_prev_uptime = currtime;
        if (rc) {
            goto err;
        }

        ii->ii_prev_uptime = currtime;

        ijo->ijo_delay = 0;
    }


    if (ijo->ijo_op == I2C_OP_WRITE) {
        /* I2C master write */
        rc = hal_i2c_master_write(ii_num, &ijo->ijo_pdata, ijo->ijo_timeout,
                                  ijo->ijo_last_op);
        if (rc) {
            goto err;
        }
    }

    if (ijo->ijo_op == I2C_OP_READ) {
        /* I2C master read */
        rc = hal_i2c_master_read(ii_num, &ijo->ijo_pdata, ijo->ijo_timeout,
                                 ijo->ijo_last_op);
        if (rc) {
            goto err;
        }
    }

    return 0;
err:
    return rc;
}

/**
 * Lock job op list
 *
 * @param The i2c job to lock the job op list for
 *
 * @return 0 on success, non-zero on failure
 */
static int
i2c_job_op_lock(struct i2c_job *ij)
{
    int rc;

    rc = os_mutex_pend(&ij->ij_job_op_lock, OS_TIMEOUT_NEVER);
    if (rc == 0 || rc == OS_NOT_STARTED) {
        return 0;
    }

    return rc;
}

/**
 * Unlock the job op list
 *
 * @param The i2c job to unlock the job list for
 *
 * @return 0 on success, non-zero on failure
 */
static void
i2c_job_op_unlock(struct i2c_job *ij)
{
    (void) os_mutex_release(&ij->ij_job_op_lock);
}

/*
 * Remove the job op from the job op list
 */
static int
i2c_remove_job_op(struct i2c_job *ij,
                  struct i2c_job_op *ijo)
{
    int rc;

    rc = i2c_job_op_lock(ij);
    if (rc) {
        goto err;
    }

    SLIST_REMOVE(&ij->ij_job_op_list, ijo, i2c_job_op, ijo_next);

    /* Free the databuf if the buffer length is greater than the buffer
     * we have
     */
    if (ijo->ijo_pdata.len > MYNEWT_VAL(I2C_FIXBUF_LEN)) {
        free(ijo->ijo_varbuf);
    }

    /* Free the job op back to the pool */
    rc = i2c_job_op_free(ijo);

    i2c_job_op_unlock(ij);

err:
    return rc;
}

/*
 * Process a job by going through each job op in the given job and remove the
 * job op after finishing
 */
static int
i2c_process_job(struct i2c_job *ij, uint8_t ii_num)
{
    int rc;
    struct i2c_job_op *cursor;
    i2c_user_arg_t *iua;

    rc = i2c_job_op_lock(ij);
    if (rc) {
        goto err;
    }

    cursor = NULL;
    SLIST_FOREACH(cursor, &ij->ij_job_op_list, ijo_next) {
        rc = i2c_process_job_op(cursor, ii_num);
        if (rc == SYS_EAGAIN) {
            /* Delayed job */
            i2c_job_op_unlock(ij);
            goto err;
        }

        if (rc) {
            break;
        }
    }

    if (ij->ij_user_func) {
        iua = ij->ij_user_arg;
        iua->iua_pdata->buffer = cursor->ijo_varbuf;
        rc = ij->ij_user_func(ij->ij_user_arg);
        if (rc) {
            i2c_job_op_unlock(ij);
            goto err;
        }
    }

    i2c_job_op_unlock(ij);

    cursor = NULL;
    SLIST_FOREACH(cursor, &ij->ij_job_op_list, ijo_next) {
        rc = i2c_remove_job_op(ij, cursor);
        if (rc) {
            goto err;
        }
    }

    return 0;
err:
    return rc;
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
 * Insert the job op into the job op list
 */
static int
i2c_insert_job_op(struct i2c_job *ij,
                  struct i2c_job_op *ijo)
{
    struct i2c_job_op *cursor, *prev;
    int rc;

    prev = NULL;

    rc = i2c_job_op_lock(ij);
    if (rc) {
        goto err;
    }

    SLIST_FOREACH(cursor, &ij->ij_job_op_list, ijo_next) {
        prev = cursor;
    }

    if (prev == NULL) {
        SLIST_INSERT_HEAD(&ij->ij_job_op_list, ijo, ijo_next);
    } else {
        SLIST_INSERT_AFTER(cursor, ijo, ijo_next);
    }

    i2c_job_op_unlock(ij);

    return 0;
err:
    return rc;
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

    os_task_init(&i2cmgr_task, "i2cmgr", i2cmgr_handler, NULL,
                 I2CMGR_TASK_PRIO, OS_WAIT_FOREVER, i2cmgr_stack,
                 I2CMGR_STACK_SIZE);


    os_mempool_init(&i2c_job_pool,
                    MYNEWT_VAL(I2C_JOB_MPOOL_MAX_NUM),
                    sizeof (struct i2c_job),
                    i2c_job_mem,
                    "i2c_job_pool");

    os_mempool_init(&i2c_job_op_pool,
                    MYNEWT_VAL(I2C_JOB_OP_MPOOL_MAX_NUM),
                    sizeof (struct i2c_job_op),
                    i2c_job_op_mem,
                    "i2c_job_op_pool");
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

/*
 * Create a job with given parameters
 *
 * @param I2C Interface number
 * @param Job priority
 * @param job callback
 * @param interface number passed as job callabck argument
 */
struct i2c_job *
i2c_create_job(uint8_t i2c_num, uint8_t prio, i2cmgr_data_func_t user_func,
               void *user_arg)
{
    struct i2c_itf *ii;
    struct i2c_job *ij;
    int rc;

    if (!user_func || !user_arg) {
        goto err;
    }

    /* Get the I2C interface with the I2C number */
    ii = i2c_get_itf_bynum(i2c_num);
    if (!ii) {
        goto err;
    }

    /* Allocate job */
    ij = i2c_job_alloc();
    if (!ij) {
        goto err;
    }

    SLIST_INIT(&ij->ij_job_op_list);

    ij->ij_prio        = prio;
    ij->ij_user_func   = user_func;
    ij->ij_user_arg    = user_arg;
    ij->ij_prev_uptime = ii->ii_prev_uptime;

    rc = os_mutex_init(&ij->ij_job_op_lock);
    if (rc) {
        free(ij);
        goto err;
    }

    return ij;
err:
    return NULL;
}

/*
 * Create a job op with given parameters
 */
static struct i2c_job_op *
i2c_create_job_op(struct hal_i2c_master_data *pdata, uint32_t timeout,
                  uint32_t delay_us, uint8_t last_op, uint8_t job_op,
                  struct i2c_job *ij)
{
    struct i2c_job_op *ijo;
    int rc;

    /* Allocate job op */
    ijo = i2c_job_op_alloc();
    if (!ijo) {
        goto err;
    }

    /* Allocate a bigger buffer if required, for most cases, the default size
     * which is 4 bytes should be enough
     */
    if (pdata->len > MYNEWT_VAL(I2C_FIXBUF_LEN)) {
        ijo->ijo_varbuf = malloc(pdata->len);
        memcpy(ijo->ijo_varbuf, pdata->buffer, pdata->len);
    } else {
        memcpy(ijo->ijo_fixbuf, pdata->buffer, pdata->len);
    }

    ijo->ijo_op      = job_op;
    ijo->ijo_pdata   = *pdata;
    ijo->ijo_last_op = last_op;
    ijo->ijo_timeout = timeout;
    ijo->ijo_delay   = delay_us;
    ijo->ijo_prev_uptime = ij->ij_prev_uptime;

    /* Insert job op */
    rc = i2c_insert_job_op(ij, ijo);
    if (rc) {
        goto err;
    }

    return ijo;
err:
    return NULL;
}

int
i2c_master_write(struct i2c_job *ij, struct hal_i2c_master_data *pdata,
                 uint32_t timeout, uint32_t delay_us, uint8_t last_op)
{
    int rc;
    struct i2c_job_op *ijo;

    /* Create job op */
    ijo = i2c_create_job_op(pdata, timeout, delay_us, last_op, I2C_OP_WRITE,
                            ij);
    if (!ijo) {
        rc = SYS_ENOENT;
        goto err;
    }

    return 0;
err:
    return rc;
}

int
i2c_master_read(struct i2c_job *ij, struct hal_i2c_master_data *pdata,
                uint32_t timeout, uint32_t delay_us,  uint8_t last_op)
{
    int rc;
    struct i2c_job_op *ijo;

    /* Create job op */
    ijo = i2c_create_job_op(pdata, timeout, delay_us, last_op, I2C_OP_READ, ij);
    if (!ijo) {
        rc = SYS_ENOENT;
        goto err;
    }

    return 0;
err:
    return rc;
}

void
i2cmgr_pkg_init(void)
{
    /* Ensure this function only gets called by sysinit. */
    SYSINIT_ASSERT_ACTIVE();

    i2cmgr_init();
}
