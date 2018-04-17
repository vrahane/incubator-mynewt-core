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

#include <inttypes.h>
#include "hal/hal_i2c.h"

#ifndef I2CMGR_H_
#define I2CMGR_H_

/**
 * Callback for I2C job, gets called after all jobs in the I2C
 * job list are executed
 *
 * @param argument
 *
 * @return 0 on success, non-zero on failure
 *
 */
typedef int (*i2cmgr_data_func_t)(void *arg);

/* Operations allowed on an I2C interface */
#define I2C_OP_READ      0
#define I2C_OP_WRITE     1
#define I2C_OP_DELAY     2

/*
 * I2C interface containing a list of jobs to be completed for current
 * interface
 */
struct i2c_itf {
    /* I2C number */
    uint8_t ii_num;
    /* Eventq ptr per interface */
    struct os_eventq *ii_evq;
    /* startup event for the interface */
    struct os_event ii_startup_ev;
    /* Task from which the I2C interface is getting accessed */
    struct os_task *ii_task;

    SLIST_ENTRY(i2c_itf) ii_next;
};

/**
 * I2C job containing a list of job ops specifying a job priority and user
 * function which would get called after job completion
 */
struct i2c_job {
    /* I2C interface number */
    uint8_t ij_itf_num;
    /* timer per job */
    struct hal_timer ij_timer;
    /* event for the job */
    struct os_event ij_ev;
    /* Job semaphore */
    struct os_sem ij_sem;
    /* I2C job data function */
    i2cmgr_data_func_t ij_cb;
    /* I2C job arg */
    void *ij_arg;
};

typedef struct {
    struct i2c_job *iua_ij;
    int iua_rc;
    struct hal_i2c_master_data *iua_pdata;
    uint8_t iua_itf_num;
    uint8_t iua_last_op;
    uint8_t iua_payload_type;
    void *iua_arg;
} i2c_user_arg_t;

/**
 * Initialize I2C manager, insert I2C interfaces, allocate stack, initialize
 * eventq for processing I2C transaction event and initialize the I2C manage
 * task
 *
 */
void i2cmgr_init(void);

/**
 * Sends a start condition and writes <ipdata->length> bytes of data on the i2c
 * bus. This API does NOT issue a stop condition unless `last_op` is set to `1`
 *
 * @param The I2C job ptr
 * @param Context containing address, databuffer and buffer length
 * @param Time to wait for transaction to complete in ticks
 * @param Delay in micro seconds to be exercised before th job op gets executed
 * @param Master should send a STOP at the end to signify end of
 *        transaction
 *
 * @return 0 on success, and non-zero error code on failure
 */
int i2c_master_write(struct i2c_job *ij, struct hal_i2c_master_data *pdata,
                     uint32_t timeout, uint32_t delay_us, uint8_t last_op);

/**
 * Sends a start condition and reads <pdata->length> bytes of data on the i2c
 * bus. This API does NOT issue a stop condition unless `last_op` is set to `1`
 *
 * @param The I2C job ptr
 * @param Context containing address, databuffer and buffer length
 * @param Time to wait for transaction to complete in ticks
 * @param Delay in micro seconds to be exercised before th job op gets executed
 * @param Master should send a STOP at the end to signify end of
 *        transaction
 *
 * @return 0 on success, and non-zero error code on failure
 */
int i2c_master_read(struct i2c_job *ij, struct hal_i2c_master_data *pdata,
                    uint32_t timeout, uint32_t delay_us, uint8_t last_op);

/*
 * Insert a job in the job list
 *
 * @param I2C interface number
 * @param I2C job
 *
 * @return 0 on success, non-zero on failure
 */
int
i2c_insert_job(uint8_t i2c_num, struct i2c_job *ij);

/*
 * Remove a job from the job list
 *
 * @param I2C interface number
 * @param I2C job
 *
 * @return 0 on success, non-zero on failure
 */
int
i2c_remove_job(uint8_t i2c_num, struct i2c_job *ij);

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
               void *user_arg);

/**
 * Puts an I2C event on the i2cmgr evq
 *
 * @param I2C mgr event context
 */
void
i2cmgr_put_evt(void *arg);

#endif /* _I2CMGR_H_ */
