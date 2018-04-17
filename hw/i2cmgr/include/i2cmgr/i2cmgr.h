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
 * Callback for the I2C job
 *
 * @param argument
 *
 * @return 0 on success, non-zero on failure
 *
 */
typedef int (*i2cmgr_data_func_t)(void *arg);

/* I2C interface */
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

/* I2C job */
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

/**
 * Initialize I2C manager, insert I2C interfaces, initialize
 * eventq for processing I2C transaction
 *
 */
void i2cmgr_init(void);

/**
 * Initialize a job
 *
 * @param job to be initilized
 *
 * @return 0 on success, non-zero on failure
 */
int i2cmgr_job_init(struct i2c_job *job);

/**
 * Blocking I2C job using i2cmgr
 *
 * @param I2C Interface number
 * @param Job to be executed
 * @param delay in usecs
 * @param job callback to be called
 * @param job callback argument
 *
 * @return 0 on success, non-zero on failure
 */
int i2cmgr_job_exec(uint8_t i2c_num, struct i2c_job *job, uint32_t delay_us,
                    i2cmgr_data_func_t job_cb, void *job_arg);

/**
 * Non-blocking I2C job using i2cmgr
 *
 * @param I2C Interface number
 * @param Job to be executed
 * @param delay in usecs
 * @param job callback to be called
 * @param job callback argument
 *
 * @return 0 on success, non-zero on failure
 */
int i2cmgr_job_noblock(uint8_t i2c_num, struct i2c_job *job, uint32_t delay_us,
                       i2cmgr_data_func_t job_cb, void *job_arg);
#endif /* _I2CMGR_H_ */
