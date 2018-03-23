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
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include "defs/error.h"
#include "os/os.h"
#include "sysinit/sysinit.h"
#include "hal/hal_i2c.h"
#include "i2cmgr/i2cmgr.h"
#include "sensor/sensor.h"
#include "ms5837/ms5837.h"
#include "sensor/temperature.h"
#include "sensor/pressure.h"
#include "ms5837_priv.h"
#include "os/os_cputime.h"
#include "console/console.h"
#include "log/log.h"
#include "stats/stats.h"

static uint16_t cnv_time[6] = {
    MS5837_CNV_TIME_OSR_256,
    MS5837_CNV_TIME_OSR_512,
    MS5837_CNV_TIME_OSR_1024,
    MS5837_CNV_TIME_OSR_2048,
    MS5837_CNV_TIME_OSR_4096,
    MS5837_CNV_TIME_OSR_8192
};

/* Define the stats section and records */
STATS_SECT_START(ms5837_stat_section)
    STATS_SECT_ENTRY(read_errors)
    STATS_SECT_ENTRY(write_errors)
    STATS_SECT_ENTRY(eeprom_crc_errors)
STATS_SECT_END

/* Define stat names for querying */
STATS_NAME_START(ms5837_stat_section)
    STATS_NAME(ms5837_stat_section, read_errors)
    STATS_NAME(ms5837_stat_section, write_errors)
    STATS_NAME(ms5837_stat_section, eeprom_crc_errors)
STATS_NAME_END(ms5837_stat_section)

/* Global variable used to hold stats data */
STATS_SECT_DECL(ms5837_stat_section) g_ms5837stats;

#define LOG_MODULE_MS5837    (5837)
#define MS5837_INFO(...)     LOG_INFO(&_log, LOG_MODULE_MS5837, __VA_ARGS__)
#define MS5837_ERR(...)      LOG_ERROR(&_log, LOG_MODULE_MS5837, __VA_ARGS__)
static struct log _log;

/* Exports for the sensor API */
static int ms5837_sensor_read(struct sensor *, sensor_type_t,
        sensor_data_func_t, void *, uint32_t);
static int ms5837_sensor_get_config(struct sensor *, sensor_type_t,
        struct sensor_cfg *);
static int ms5837_sensor_set_config(struct sensor *, void *);

static const struct sensor_driver g_ms5837_sensor_driver = {
    .sd_read = ms5837_sensor_read,
    .sd_get_config = ms5837_sensor_get_config,
    .sd_set_config = ms5837_sensor_set_config,
};

struct iua_read_arg {
    struct sensor *ira_sensor;
    sensor_data_func_t ira_data_func;
    void *ira_data_arg;
};

i2c_user_arg_t read_arg;
i2c_user_arg_t write_arg;
struct iua_read_arg read_arg_ira;


/**
 * Expects to be called back through os_dev_create().
 *
 * @param The device object associated with ms5837
 * @param Argument passed to OS device init, unused
 *
 * @return 0 on success, non-zero error on failure.
 */
int
ms5837_init(struct os_dev *dev, void *arg)
{
    struct ms5837 *ms5837;
    struct sensor *sensor;
    struct sensor_itf *itf;
    int rc;

    if (!arg || !dev) {
        rc = SYS_ENODEV;
        goto err;
    }

    ms5837 = (struct ms5837 *)dev;

    log_register(dev->od_name, &_log, &log_console_handler, NULL, LOG_SYSLEVEL);

    sensor = &ms5837->sensor;

    itf = SENSOR_GET_ITF(sensor);

    /* Initialise the stats entry */
    rc = stats_init(
        STATS_HDR(g_ms5837stats),
        STATS_SIZE_INIT_PARMS(g_ms5837stats, STATS_SIZE_32),
        STATS_NAME_INIT_PARMS(ms5837_stat_section));
    SYSINIT_PANIC_ASSERT(rc == 0);
    /* Register the entry with the stats registry */
    rc = stats_register(dev->od_name, STATS_HDR(g_ms5837stats));
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = sensor_init(sensor, dev);
    if (rc != 0) {
        goto err;
    }

    /* Add the driver with all the supported type */
    rc = sensor_set_driver(sensor, SENSOR_TYPE_AMBIENT_TEMPERATURE |
                           SENSOR_TYPE_PRESSURE,
                           (struct sensor_driver *)&g_ms5837_sensor_driver);
    if (rc) {
        goto err;
    }

    /* Set the interface */
    rc = sensor_set_interface(sensor, arg);
    if (rc) {
        goto err;
    }

    rc = sensor_mgr_register(sensor);
    if (rc) {
        goto err;
    }

    rc = ms5837_read_eeprom(itf, ms5837->pdd.eeprom_coeff);
    if (rc) {
        goto err;
    }

    return 0;
err:
    return rc;

}

static int
ms5837_sensor_read(struct sensor *sensor, sensor_type_t type,
        sensor_data_func_t data_func, void *data_arg, uint32_t timeout)
{
    uint32_t rawtemp;
    uint32_t rawpress;
    int32_t comptemp;
    int32_t deltat;
    float temperature;
    float pressure;
    struct sensor_itf *itf;
    struct ms5837 *ms5837;
    struct ms5837_cfg *cfg;

    int rc;
    union {
        struct sensor_temp_data std;
        struct sensor_press_data spd;
    } databuf;

    if (!(type & SENSOR_TYPE_PRESSURE)    &&
        !(type & SENSOR_TYPE_AMBIENT_TEMPERATURE)) {
        rc = SYS_EINVAL;
        goto err;
    }

    itf = SENSOR_GET_ITF(sensor);

    ms5837 = (struct ms5837 *)SENSOR_GET_DEVICE(sensor);

    cfg = &(ms5837->cfg);

    temperature = pressure = 0;

    read_arg_ira.ira_data_func = data_func;
    read_arg_ira.ira_data_arg  = data_arg;
    read_arg_ira.ira_sensor    = sensor;

    /* Get a new pressure sample */
    if (type & SENSOR_TYPE_PRESSURE) {
        rc = ms5837_get_rawtemp(itf, &rawtemp, cfg->mc_s_temp_res_osr);
        if (rc) {
            goto err;
        }

        rc = ms5837_get_rawpress(itf, &rawpress, cfg->mc_s_press_res_osr);
        if (rc) {
            goto err;
        }

#if !MYNEWT_VAL(USE_I2CMGR)
        /* compensate using temperature and pressure coefficients
         * competemp is the first order compensated temperature
         * which is used as input to the pressure compensation
         */
        temperature = ms5837_compensate_temperature(ms5837->pdd.eeprom_coeff, rawtemp,
                                                    &comptemp, &deltat);

        pressure = ms5837_compensate_pressure(ms5837->pdd.eeprom_coeff, comptemp,
                                              rawpress, deltat);

        databuf.spd.spd_press = pressure;
        databuf.spd.spd_press_is_valid = 1;

        /* Call data function */
        rc = data_func(sensor, data_arg, &databuf.spd, SENSOR_TYPE_PRESSURE);
        if (rc) {
            goto err;
        }
#endif
    }

    /* Get a new temperature sample */
    if (type & SENSOR_TYPE_AMBIENT_TEMPERATURE) {
        if (!temperature) {
            rc = ms5837_get_rawtemp(itf, &rawtemp, cfg->mc_s_temp_res_osr);
            if (rc) {
                goto err;
            }
#if !MYNEWT_VAL(USE_I2CMGR)
            temperature = ms5837_compensate_temperature(ms5837->pdd.eeprom_coeff, rawtemp,
                                                        NULL, NULL);
        }

        databuf.std.std_temp = temperature;
        databuf.std.std_temp_is_valid = 1;

        /* Call data function */
        rc = data_func(sensor, data_arg, &databuf.std,
                       SENSOR_TYPE_AMBIENT_TEMPERATURE);
        if (rc) {
            goto err;
        }
#else
        }
#endif
    }

    return 0;
err:
    return rc;
}

static int
ms5837_sensor_get_config(struct sensor *sensor, sensor_type_t type,
        struct sensor_cfg *cfg)
{
    int rc;

    if (!(type & SENSOR_TYPE_PRESSURE) ||
        !(type & SENSOR_TYPE_AMBIENT_TEMPERATURE)) {
        rc = SYS_EINVAL;
        goto err;
    }

    cfg->sc_valtype = SENSOR_VALUE_TYPE_FLOAT;

    return (0);
err:
    return (rc);
}

static int
ms5837_sensor_set_config(struct sensor *sensor, void *cfg)
{
    struct ms5837* ms5837 = (struct ms5837 *)SENSOR_GET_DEVICE(sensor);

    return ms5837_config(ms5837, (struct ms5837_cfg*)cfg);
}

/**
 * Configure MS5837 sensor
 *
 * @param Sensor device MS5837 structure
 * @param Sensor device MS5837 config
 *
 * @return 0 on success, non-zero on failure
 */
int
ms5837_config(struct ms5837 *ms5837, struct ms5837_cfg *cfg)
{
    int rc;
    struct sensor_itf *itf;

    itf = SENSOR_GET_ITF(&(ms5837->sensor));

    rc = ms5837_reset(itf);
    if (rc) {
        goto err;
    }

    rc = sensor_set_type_mask(&(ms5837->sensor),  cfg->mc_s_mask);
    if (rc) {
        goto err;
    }

    ms5837->cfg.mc_s_temp_res_osr = cfg->mc_s_temp_res_osr;

    ms5837->cfg.mc_s_press_res_osr = cfg->mc_s_press_res_osr;

    ms5837->cfg.mc_s_mask = cfg->mc_s_mask;

    return 0;
err:
    return (rc);
}


static int
ms5837_i2c_writelen_done(void *arg)
{
    i2c_user_arg_t *wa = arg;

    if (!wa->iua_rc) {
        console_printf("ms5837 write done\n");
    } else {
        console_printf("ms5837 write failed\n");
        MS5837_ERR("I2C manager command write failed at address 0x%02X at job"
                   "0x%08x for I2C number %u\n", wa->iua_pdata->address,
                    wa->iua_ij, wa->iua_itf_num);
        STATS_INC(g_ms5837stats, write_errors);
    }

    return wa->iua_rc;
}

static int
ms5837_i2c_readlen_done(void *arg)
{
    int rc;
    int idx;
    uint8_t *data;
    float pressure;
    float temperature;
    struct ms5837 *ms5837;
    i2c_user_arg_t *ra = arg;
    uint16_t coeff[MS5837_NUMBER_COEFFS];
    union {
        struct sensor_temp_data std;
        struct sensor_press_data spd;
    } databuf;

    if (!ra->iua_rc) {
        console_printf("ms5827 read done\n");
    } else {
        console_printf("ms5837 read failed\n");
        MS5837_ERR("I2C manager command read failed at address 0x%02X at job"
                   "0x%08x for I2C number %u\n", ra->iua_pdata->address,
                    ra->iua_ij, ra->iua_itf_num);
        STATS_INC(g_ms5837stats, read_errors);
    }

    data = wa->iua_pdata->buffer;
    ira = (struct iua_read_arg *)ra->arg;
    ms5837 = (struct ms5837 *)SENSOR_GET_DEVICE(ira->ira_sensor);


    switch(wa->iua_payload_type) {
        case MS5837_RAW_DATA:
            console_printf("ms5827 raw data \n");
        case MS5837_TEMP_DATA:
            console_printf("ms5827 temp data \n");
            *data = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
            if (!temperature) {
                temperature = ms5837_compensate_temperature(
                    ms5837->pdd.eeprom_coeff, rawtemp, NULL, NULL);

                databuf.std.std_temp = temperature;
                databuf.std.std_temp_is_valid = 1;

                /* Call data function */
                rc = ira->ira_data_func(ira->ira_sensor, ira->ira_data_arg,
                                        &databuf.std,
                                        SENSOR_TYPE_AMBIENT_TEMPERATURE);
                if (rc) {
                    goto err;
                }
            }
            break;
        case MS5837_PRESS_DATA:
            console_printf("ms5827 press data \n");
            *data = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
            temperature = ms5837_compensate_temperature(
                ms5837->pdd.eeprom_coeff, rawtemp, &comptemp, &deltat);

            pressure = ms5837_compensate_pressure(ms5837->pdd.eeprom_coeff,
                comptemp, rawpress, deltat);

            databuf.spd.spd_press = pressure;
            databuf.spd.spd_press_is_valid = 1;

            /* Call data function */
            rc = ira->ira_data_func(ira->ira_sensor, ira->ira_data_arg,
                                    &databuf.spd, SENSOR_TYPE_PRESSURE);
            if (rc) {
                goto err;
            }
            break;
        case MS5837_EEPROM:
            console_printf("ms5827 EEPROM data \n");
            for(idx = 0; idx < MS5837_NUMBER_COEFFS; idx++) {
                data[idx] = (((data[idx] & 0xFF00) >> 8)|
                            ((data[idx] & 0x00FF) << 8));
            }
            rc = ms5837_crc_check(data, (data[MS5837_IDX_CRC] & 0xF000) >> 12);
            if (rc) {
                rc = SYS_EINVAL;
                // MS5837_ERR("Failure in CRC, 0x%02X\n", payload[idx]);
                STATS_INC(g_ms5837stats, eeprom_crc_errors);
                goto err;
            }

            memcpy(coeff, data, sizeof(data));
            break;
        default:
            break;
    }

    return ra->iua_rc;
}

/**
 * Write multiple length data to MS5837 sensor over I2C
 *
 * @param The sensor interface
 * @param register address
 * @param variable length payload
 * @param length of the payload to write
 * @param delay in micro seconds
 *
 * @return 0 on success, non-zero on failure
 */
int
ms5837_writelen(struct sensor_itf *itf, uint8_t addr, uint8_t *buffer,
                uint8_t len, uint32_t delay)
{
    int rc;

    struct hal_i2c_master_data data_struct = {
        .address = itf->si_addr,
        .len = 1,
        .buffer = &addr
    };

    /* Register write */
#if MYNEWT_VAL(USE_I2CMGR)
    struct i2c_job *ij;
    ij = i2c_create_job(itf->si_num, 0, ms5837_i2c_writelen_done,
                        (void *)&write_arg);
    if (!ij) {
        rc = SYS_ENOMEM;
        goto err;
    }

    rc = i2c_master_write(ij, &data_struct, OS_TICKS_PER_SEC / 10, 0, 1);
    if (rc) {
        goto err;
    }

    rc = i2c_insert_job(itf->si_num, ij);
    if (rc) {
        goto err;
    }
#else
    rc = hal_i2c_master_write(itf->si_num, &data_struct, OS_TICKS_PER_SEC / 10, 0, 1);
    if (rc) {
        MS5837_ERR("I2C write command write failed at address 0x%02X\n",
                   data_struct.address);
        STATS_INC(g_ms5837stats, write_errors);
        goto err;
    }
#endif

    return 0;
err:
    return rc;
}

/**
 * Read multiple length data from MS5837 sensor over I2C
 *
 * @param The sensor interface
 * @param register address
 * @param variable length buffer
 * @param length of the payload to read
 * @param Delay in micro seconds
 *
 * @return 0 on success, non-zero on failure
 */
int
ms5837_readlen(struct sensor_itf *itf, uint8_t addr, uint8_t *buffer,
               uint8_t len, uint32_t delay)
{
    int rc;
    uint8_t payload[3] = {addr, 0, 0};

    struct hal_i2c_master_data data_struct = {
        .address = itf->si_addr,
        .len = 1,
        .buffer = payload
    };

    /* Clear the supplied buffer */
    memset(buffer, 0, len);

    /* Command write */
#if MYNEWT_VAL(USE_I2CMGR)
    struct i2c_job *ij;

    ij = i2c_create_job(itf->si_num, 0, ms5837_i2c_readlen_done,
                        (void *)&read_arg);
    if (!ij) {
        rc = SYS_ENOMEM;
        goto err;
    }

    rc = i2c_master_write(ij, &data_struct, OS_TICKS_PER_SEC / 10, 0, 1);
    if (rc) {
        goto err;
    }

#else
    /* delay conversion depending on resolution */
    os_cputime_delay_usecs(delay);

    rc = hal_i2c_master_write(itf->si_num, &data_struct, OS_TICKS_PER_SEC / 10, 0, 1);
    if (rc) {
        MS5837_ERR("I2C read command write failed at address 0x%02X\n",
                   data_struct.address);
        STATS_INC(g_ms5837stats, write_errors);
        goto err;
    }
#endif

    /* Read len bytes back */
    memset(payload, 0, sizeof(payload));
    data_struct.len = len;

#if MYNEWT_VAL(USE_I2CMGR)
    rc = i2c_master_read(ij, &data_struct, OS_TICKS_PER_SEC / 10, delay, 1);
    if (rc) {
        goto err;
    }

    rc = i2c_insert_job(itf->si_num, ij);
    if (rc) {
        goto err;
    }
#else
    rc = hal_i2c_master_read(itf->si_num, &data_struct, OS_TICKS_PER_SEC / 10, 0, 1);
    if (rc) {
        MS5837_ERR("Failed to read from 0x%02X:0x%02X\n", data_struct.address, addr);
        STATS_INC(g_ms5837stats, read_errors);
        goto err;
    }

    /* Copy the I2C results into the supplied buffer */
    memcpy(buffer, payload, len);
#endif

    return 0;
err:
    return rc;
}

/**
 * Reads the ms5837 EEPROM coefficients for computation and
 * does a CRC check on them
 *
 * @param the sensor interface
 * @param buffer to fill up the coefficients
 *
 * @return 0 on success, non-zero on failure
 */
int
ms5837_read_eeprom(struct sensor_itf *itf, uint16_t *coeff)
{
    int idx;
    int rc;
    uint16_t payload[MS5837_NUMBER_COEFFS];

    for(idx = 0; idx < MS5837_NUMBER_COEFFS; idx++) {
        rc = ms5837_readlen(itf, MS5837_CMD_PROM_READ_ADDR0 + idx * 2,
                            (uint8_t *)(payload + idx), 2, 0);
        if (rc) {
            goto err;
        }

#if !MYNEWT_VAL(USE_I2CMGR)
        payload[idx] = (((payload[idx] & 0xFF00) >> 8)|
                        ((payload[idx] & 0x00FF) << 8));
    }

    rc = ms5837_crc_check(payload, (payload[MS5837_IDX_CRC] & 0xF000) >> 12);
    if (rc) {
        rc = SYS_EINVAL;
        // MS5837_ERR("Failure in CRC, 0x%02X\n", payload[idx]);
        STATS_INC(g_ms5837stats, eeprom_crc_errors);
        goto err;
    }

    memcpy(coeff, payload, sizeof(payload));
#else
        read_arg.iua_payload_type = MS5837_EEPROM;
    }
#endif

    return 0;
err:
    return rc;
}

/**
 * Compensate for pressure using coefficients from the EEPROM
 *
 * @param ptr to coefficients
 * @param first order compensated temperature
 * @param raw pressure
 * @param deltat temperature
 *
 * @return second order temperature compensated pressure
 */
float
ms5837_compensate_pressure(uint16_t *coeffs, int32_t temp,
                           uint32_t rawpress, int32_t deltat)
{
    int64_t off, sens, off2, sens2;

    off2 = sens2 = 0;

    /* off = off_T1 + TCO * dt */
    off = ((int64_t)(coeffs[MS5837_IDX_PRESS_OFF]) << 17) +
          (((int64_t)(coeffs[MS5837_IDX_TEMP_COEFF_PRESS_OFF]) * deltat) >> 6);

    /* sensitivity at actual temperature = sens_T1 + TCS * dt */
    sens = ((int64_t)coeffs[MS5837_IDX_PRESS_SENS] << 16) +
           (((int64_t)coeffs[MS5837_IDX_TEMP_COEFF_PRESS_SENS] * deltat) >> 7);

    /* second order temperature compensation */
    if(temp < 2000) {
        /* low temperature */
        off2 = (31 * ((int64_t)temp - 2000) * ((int64_t)temp - 2000)) >> 3;
        sens2 = (63 * ((int64_t)temp - 2000) * ((int64_t)temp - 2000)) >> 5;
    }

    off2  = off - off2;

    sens2 = sens - sens2;

    /* temperature compensated second order pressure = D1 * sens - off */
    return ((float)(((rawpress * sens2) >> 21) - off2)/32768);
}

/**
 * Compensate for temperature using coefficients from the EEPROM
 *
 * @param ptr to coefficients
 * @param compensated temperature
 * @param raw temperature
 * @param optional ptr to fill up first order compensated temperature
 * @param optional ptr to fill up delta temperature
 *
 * @return second order temperature compensated temperature
 */
float
ms5837_compensate_temperature(uint16_t *coeffs, uint32_t rawtemp,
                              int32_t *comptemp, int32_t *deltat)
{
    int32_t dt, temp;
    int64_t t2;

    t2 = 0;

    /* difference between actual and reference temperature = D2 - Tref */
    dt = (int32_t)rawtemp - ((int32_t)coeffs[MS5837_IDX_REF_TEMP] << 8);

    /* actual temperature = 2000 + dt * tempsens */
    temp = 2000 + (dt * coeffs[MS5837_IDX_TEMP_COEFF_TEMP] >> 23);

    if (comptemp) {
        *comptemp = temp;
    }

    if (deltat) {
        *deltat = dt;
    }

    if(temp < 2000) {
        /* low temperature */
        t2 = (11 * (int64_t)dt  * (int64_t)dt) >> 35;
    }

    /* second order temperature */
    return (((float)temp - t2)/100);
}

/**
 * Triggers conversion and reads ADC value
 *
 * @param the sensor interface
 * @param cmd used for conversion, considers temperature, pressure and OSR
 * @param ptr to ADC value
 *
 * @return 0 on success, non-zero on failure
 */
static int
ms5837_get_raw_data(struct sensor_itf *itf, uint8_t cmd, uint32_t *data)
{
    int rc;
    uint8_t payload[3] = {0};

    /* send conversion command based on OSR, temperature and pressure */
    rc = ms5837_writelen(itf, cmd, payload, 0, 0);
    if (rc) {
        goto err;
    }

    /* read adc value */
    rc = ms5837_readlen(itf, MS5837_CMD_ADC_READ, payload, 3,
                        cnv_time[(cmd & MS5837_CNV_OSR_MASK)/2]);
    if (rc) {
        goto err;
    }

#if !MYNEWT_VAL(USE_I2CMGR)
    *data = ((uint32_t)payload[0] << 16) | ((uint32_t)payload[1] << 8) | payload[2];
#else
    read_arg.iua_payload_type = MS5837_RAW_DATA;
#endif

    return 0;
err:
    return rc;
}

/**
 * Reads the temperature ADC value
 *
 * @param the sensor interface
 * @param raw adc temperature value
 * @param resolution osr
 *
 * @return 0 on success, non-zero on failure
 */
int
ms5837_get_rawtemp(struct sensor_itf *itf, uint32_t *rawtemp,
                   uint8_t res_osr)
{
    uint8_t cmd;
    uint32_t tmp;
    int rc;

    /* read temperature ADC value */
    cmd = res_osr | MS5837_CMD_TEMP;
    rc = ms5837_get_raw_data(itf, cmd, &tmp);
    if (rc) {
        goto err;
    }

#if !MYNEWT_VAL(USE_I2CMGR)
    *rawtemp = tmp;
#else
    read_arg.iua_payload_type = MS5837_TEMP_DATA;
#endif

    return 0;
err:
    return rc;
}

/**
 * Reads the pressure ADC value
 *
 * @param the sensor interface
 * @param raw adc pressure value
 * @param resolution osr
 *
 * @return 0 on success, non-zero on failure
 */
int
ms5837_get_rawpress(struct sensor_itf *itf, uint32_t *rawpress,
                    uint8_t res_osr)
{
    uint8_t cmd;
    uint32_t tmp;
    int rc;

    /* read pressure ADC value */
    cmd = res_osr | MS5837_CMD_PRESS;
    rc = ms5837_get_raw_data(itf, cmd, &tmp);
    if (rc) {
        goto err;
    }

#if !MYNEWT_VAL(USE_I2CMGR)
    *rawpress = tmp;
#else
    read_arg.iua_payload_type = MS5837_PRESS_DATA;
#endif

    return 0;
err:
    return rc;
}

/**
 * Resets the MS5837 chip
 *
 * @param the sensor interface
 *
 * @return 0 on success, non-zero on failure
 */
int
ms5837_reset(struct sensor_itf *itf)
{
    uint8_t txdata;

    txdata = 0;

    return ms5837_writelen(itf, MS5837_CMD_RESET, &txdata, 0, 0);
}

/**
 * crc4 check for MS5837 EEPROM
 *
 * @param buffer containing EEPROM coefficients
 * @param crc to compare with
 *
 * return 0 on success (CRC is OK), non-zero on failure
 */
int
ms5837_crc_check(uint16_t *prom, uint8_t crc)
{
    uint8_t cnt, bit;
    uint16_t rem, crc_read;

    rem = 0x00;
    crc_read = prom[0];
    prom[MS5837_NUMBER_COEFFS] = 0;

    /* Clear the CRC byte */
    prom[0] = (0x0FFF & prom[0]);

    for(cnt = 0; cnt < (MS5837_NUMBER_COEFFS + 1) * 2; cnt++) {
        /* Get next byte */
        if (cnt%2 == 1) {
            rem ^=  (prom[cnt>>1] & 0x00FF);
        } else {
            rem ^=  (prom[cnt>>1] >> 8);
        }

        for(bit = 8; bit > 0; bit--) {
            if(rem & 0x8000) {
                rem = (rem << 1) ^ 0x3000;
            } else {
                rem <<= 1;
            }
        }
    }

    rem >>= 12;
    prom[0] = crc_read;

    return  (rem != crc);
}
