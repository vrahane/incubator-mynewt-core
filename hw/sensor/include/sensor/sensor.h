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

#ifndef __SENSOR_H__
#define __SENSOR_H__

#include <string.h>
#include "os/os.h"
#include "os/os_dev.h"
#include "syscfg/syscfg.h"

#if MYNEWT_VAL(SENSOR_OIC)
#include "oic/oc_ri.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Package init function.  Remove when we have post-kernel init stages.
 */
void sensor_pkg_init(void);


/* Forward declare sensor structure defined below. */
struct sensor;

/**
 * @{ Sensor API
 */

typedef enum {
 /* No sensor type, used for queries */
    SENSOR_TYPE_NONE                 = 0,
    /* Accelerometer functionality supported */
    SENSOR_TYPE_ACCELEROMETER        = (1 << 0),
    /* Magnetic field supported */
    SENSOR_TYPE_MAGNETIC_FIELD       = (1 << 1),
    /* Gyroscope supported */
    SENSOR_TYPE_GYROSCOPE            = (1 << 2),
    /* Light supported */
    SENSOR_TYPE_LIGHT                = (1 << 3),
    /* Temperature supported */
    SENSOR_TYPE_TEMPERATURE          = (1 << 4),
    /* Ambient temperature supported */
    SENSOR_TYPE_AMBIENT_TEMPERATURE  = (1 << 5),
    /* Pressure sensor supported */
    SENSOR_TYPE_PRESSURE             = (1 << 6),
    /* Proximity sensor supported */
    SENSOR_TYPE_PROXIMITY            = (1 << 7),
    /* Relative humidity supported */
    SENSOR_TYPE_RELATIVE_HUMIDITY    = (1 << 8),
    /* Rotation vector (quaternion) supported */
    SENSOR_TYPE_ROTATION_VECTOR      = (1 << 9),
    /* Altitude Supported */
    SENSOR_TYPE_ALTITUDE             = (1 << 10),
    /* Weight Supported */
    SENSOR_TYPE_WEIGHT               = (1 << 11),
    /* Linear Accelerometer (Without Gravity) */
    SENSOR_TYPE_LINEAR_ACCEL         = (1 << 12),
    /* Gravity Sensor */
    SENSOR_TYPE_GRAVITY              = (1 << 13),
    /* Euler Orientation Sensor */
    SENSOR_TYPE_EULER                = (1 << 14),
    /* Color Sensor */
    SENSOR_TYPE_COLOR                = (1 << 15),

    /* Standard sensor types to be defined here */

    /* User defined sensor type 1 */
    SENSOR_TYPE_USER_DEFINED_1       = (1 << 26),
    /* User defined sensor type 2 */
    SENSOR_TYPE_USER_DEFINED_2       = (1 << 27),
    /* User defined sensor type 3 */
    SENSOR_TYPE_USER_DEFINED_3       = (1 << 28),
    /* User defined sensor type 4 */
    SENSOR_TYPE_USER_DEFINED_4       = (1 << 29),
    /* User defined sensor type 5 */
    SENSOR_TYPE_USER_DEFINED_5       = (1 << 30),
    /* User defined sensor type 6 */
    SENSOR_TYPE_USER_DEFINED_6       = (1 << 31),
    /* A selector, describes all sensors */
    SENSOR_TYPE_ALL                  = 0xFFFFFFFF
} sensor_type_t;

typedef enum {
    /* Accelerometer double tap event */
    SENSOR_EVENT_TYPE_DOUBLE_TAP     = (1 << 0),
    /* Accelerometer single tap event */
    SENSOR_EVENT_TYPE_SINGLE_TAP     = (1 << 1),
} sensor_event_type_t;


/**
 * Opaque 32-bit value, must understand underlying sensor type
 * format in order to interpret.
 */
#define SENSOR_VALUE_TYPE_OPAQUE (0)
/**
 * 32-bit signed integer
 */
#define SENSOR_VALUE_TYPE_INT32  (1)
/**
 * 32-bit floating point
 */
#define SENSOR_VALUE_TYPE_FLOAT  (2)
/**
 * 32-bit integer triplet.
 */
#define SENSOR_VALUE_TYPE_INT32_TRIPLET (3)
/**
 * 32-bit floating point number triplet.
 */
#define SENSOR_VALUE_TYPE_FLOAT_TRIPLET (4)

/**
 * Sensor interfaces
 */
#define SENSOR_ITF_SPI    (0)
#define SENSOR_ITF_I2C    (1)
#define SENSOR_ITF_UART   (2)

/**
 * Sensor threshold constants
 */
#define SENSOR_THRESH_ALGO_WINDOW     0x1
#define SENSOR_THRESH_ALGO_WATERMARK  0x2
#define SENSOR_THRESH_ALGO_USERDEF    0x3

/**
 * Sensor listener constants
 */
#define SENSOR_IGN_LISTENER     1

/**
 * Useful constants
 */
#define STANDARD_ACCEL_GRAVITY 9.80665F

/**
 * Configuration structure, describing a specific sensor type off of
 * an existing sensor.
 */
struct sensor_cfg {
    /* The value type for this sensor (e.g. SENSOR_VALUE_TYPE_INT32).
     * Used to describe the result format for the value corresponding
     * to a specific sensor type.
     */
    uint8_t sc_valtype;
    /* Reserved for future usage */
    uint8_t _reserved[3];
};

typedef union {
    struct sensor_mag_data   *smd;
    struct sensor_accel_data *sad;
    struct sensor_euler_data *sed;
    struct sensor_quat_data  *sqd;
    struct sensor_accel_data *slad;
    struct sensor_accel_data *sgrd;
    struct sensor_gyro_data  *sgd;
    struct sensor_temp_data  *std;
    struct sensor_temp_data  *satd;
    struct sensor_light_data *sld;
    struct sensor_color_data *scd;
    struct sensor_press_data *spd;
    struct sensor_humid_data *srhd;
}sensor_data_t;

/**
 * Callback for handling sensor data, specified in a sensor listener.
 *
 * @param The sensor for which data is being returned
 * @param The argument provided to sensor_read() function.
 * @param A single sensor reading for that sensor listener
 * @param The sensor type for the data function
 *
 * @return 0 on success, non-zero error code on failure.
 */
typedef int (*sensor_data_func_t)(struct sensor *, void *, void *,
             sensor_type_t);

/**
 * Callback for sending trigger notification.
 *
 * @param ptr to the sensor
 * @param ptr to sensor data
 * @param the sensor type
 */
typedef int
(*sensor_trigger_notify_func_t)(struct sensor *, void *, sensor_type_t);

/**
 * Callback for trigger compare functions.
 *
 * @param type of sensor
 * @param the sensor low threshold
 * @param the sensor high threshold
 * @param ptr to data
 */

typedef int
(*sensor_trigger_cmp_func_t)(sensor_type_t, sensor_data_t *,
                             sensor_data_t *, void *);

/**
 * Callback for event notifications.
 *
 * @param The sensor that observed the event
 * @param The opaque argument provided during registration
 * @param The sensor event type that was observed
 */
typedef int
(*sensor_notifier_func_t)(struct sensor *, void *, sensor_event_type_t);

/**
 *
 */
struct sensor_listener {
    /* The type of sensor data to listen for, this is interpreted as a
     * mask, and this listener is called for all sensor types on this
     * sensor that match the mask.
     */
    sensor_type_t sl_sensor_type;

    /* Sensor data handler function, called when has data */
    sensor_data_func_t sl_func;

    /* Argument for the sensor listener */
    void *sl_arg;

    /* Next item in the sensor listener list.  The head of this list is
     * contained within the sensor object.
     */
    SLIST_ENTRY(sensor_listener) sl_next;
};

/**
 * Registration for sensor event notifications
 */
struct sensor_notifier {
    /* The type of sensor event to be notified on, this is interpreted as a
     * mask, and this notifier is called for all event types on this
     * sensor that match the mask.
     */
    sensor_event_type_t sn_sensor_event_type;

    /* Sensor event notification handler function, called when a matching
     * event occurred.
     */
    sensor_notifier_func_t sn_func;

    /* Opaque argument for the sensor event notification handler function. */
    void *sn_arg;

    /* Next item in the sensor notifier list. The head of this list is
     * contained within the sensor object.
     */
    SLIST_ENTRY(sensor_notifier) sn_next;
};

/**
 * Context for sensor read events
 */
struct sensor_read_ev_ctx {
    /* The sensor for which the ev cb should be called */
    struct sensor *srec_sensor;
    /* The sensor type */
    sensor_type_t srec_type;
};

/**
 * Sensor type traits list
 */
struct sensor_type_traits {
    /* The type of sensor data for checking against thresholds */
    sensor_type_t stt_sensor_type;

    /* Low threshold per sensor type */
    sensor_data_t stt_low_thresh;

    /* High threshold per sensor type */
    sensor_data_t stt_high_thresh;

    /* field for selecting algorithm */
    uint8_t stt_algo;

    /* Poll rate multiple */
    uint16_t stt_poll_n;

    uint16_t stt_polls_left;

#if MYNEWT_VAL(SENSOR_POLL_TEST_LOG)
    os_time_t prev_now;
#endif

    /* function ptr for setting comparison algo */
    sensor_trigger_cmp_func_t stt_trigger_cmp_algo;

#if MYNEWT_VAL(SENSOR_OIC)
    /* Sensor OIC resource */
    oc_resource_t *stt_oic_res;
#endif

    struct sensor *stt_sensor;

    /* Next item in the sensor traits list.  The head of this list is
     * contained within the sensor object.
     */
    SLIST_ENTRY(sensor_type_traits) stt_next;
};

struct sensor_notify_ev_ctx {
    /* The sensor for which the ev cb should be called */
    struct sensor * snec_sensor;
    /* The event type */
    sensor_event_type_t snec_evtype;
};

/**
 * Read a single value from a sensor, given a specific sensor type
 * (e.g. SENSOR_TYPE_PROXIMITY).
 *
 * @param The sensor to read from
 * @param The type(s) of sensor values to read.  Mask containing that type, provide
 *        all, to get all values.
 * @param The function to call with each value read.  If NULL, it calls all
 *        sensor listeners associated with this function.
 * @param The argument to pass to the read callback.
 * @param Timeout.  If block until result, specify OS_TIMEOUT_NEVER, 0 returns
 *        immediately (no wait.)
 *
 * @return 0 on success, non-zero error code on failure.
 */
typedef int (*sensor_read_func_t)(struct sensor *, sensor_type_t,
        sensor_data_func_t, void *, uint32_t);

/**
 * Get the configuration of the sensor for the sensor type.  This includes
 * the value type of the sensor.
 *
 * @param The type of sensor value to get configuration for
 * @param A pointer to the sensor value to place the returned result into.
 *
 * @return 0 on success, non-zero error code on failure.
 */
typedef int (*sensor_get_config_func_t)(struct sensor *, sensor_type_t,
        struct sensor_cfg *);

/** 
 * Send a new configuration register set to the sensor.
 *
 * @param ptr to the sensor-specific stucture
 * @param ptr to the sensor-specific configuration structure
 *
 * @return 0 on success, non-zero error code on failure.
 */
typedef int (*sensor_set_config_func_t)(struct sensor *, void *);

/**
 * Set the trigger and threshold values for a specific sensor for the sensor
 * type.
 *
 * @param ptr to the sensor
 * @param type of sensor
 * @param low threshold
 * @param high threshold
 *
 * @return 0 on success, non-zero error code on failure.
 */
typedef int (*sensor_set_trigger_thresh_t)(struct sensor *, sensor_type_t,
                                           struct sensor_type_traits *stt);

/**
 * Clear the high/low threshold values for a specific sensor for the sensor
 * type.
 *
 * @param ptr to the sensor
 * @param type of sensor
 *
 * @return 0 on success, non-zero error code on failure.
 */
typedef int (*sensor_clear_trigger_thresh_t)(struct sensor *, sensor_type_t);

/**
 * Set the notification expectation for a targeted set of events for the
 * specific sensor. After this function returns successfully, the implementer
 * shall post corresponding event notifications to the sensor manager.
 *
 * @param The sensor to expect notifications from.
 * @param The mask of event types to expect notifications from.
 *
 * @return 0 on success, non-zero error code on failure.
 */
typedef int (*sensor_set_notification_t)(struct sensor *,
                                         sensor_event_type_t);

/**
 * Unset the notification expectation for a targeted set of events for the
 * specific sensor.
 *
 * @param The sensor.
 * @param The mask of event types.
 *
 * @return 0 on success, non-zero error code on failure.
 */
typedef int (*sensor_unset_notification_t)(struct sensor *,
                                           sensor_event_type_t);

/**
 * Let driver handle interrupt in the sensor context
 *
 * @param The sensor.
 * @param Interrupt argument
 *
 * @return 0 on success, non-zero error code on failure.
 */
typedef int (*sensor_handle_interrupt_t)(struct sensor *);

struct sensor_driver {
    sensor_read_func_t sd_read;
    sensor_get_config_func_t sd_get_config;
    sensor_set_config_func_t sd_set_config;
    sensor_set_trigger_thresh_t sd_set_trigger_thresh;
    sensor_clear_trigger_thresh_t sd_clear_low_trigger_thresh;
    sensor_clear_trigger_thresh_t sd_clear_high_trigger_thresh;
    sensor_set_notification_t sd_set_notification;
    sensor_unset_notification_t sd_unset_notification;
    sensor_handle_interrupt_t sd_handle_interrupt;
};

struct sensor_timestamp {
    struct os_timeval st_ostv;
    struct os_timezone st_ostz;
    uint32_t st_cputime;
};

struct sensor_int {
    uint8_t host_pin;
    uint8_t device_pin;
    uint8_t active;
};

struct sensor_itf {

    /* Sensor interface type */
    uint8_t si_type;

    /* Sensor interface number */
    uint8_t si_num;

    /* Sensor CS pin */
    uint8_t si_cs_pin;

    /* Sensor address */
    uint16_t si_addr;

    /* Sensor interface low int pin */
    uint8_t si_low_pin;

    /* Sensor interface high int pin */
    uint8_t si_high_pin;

    /* Sensor interface interrupts pins */
    /* XXX We should probably remove low/high pins and replace it with those
     */
    struct sensor_int si_ints[MYNEWT_VAL(SENSOR_MAX_INTERRUPTS_PINS)];
};

/*
 * Return the OS device structure corresponding to this sensor
 */
#define SENSOR_GET_DEVICE(__s) ((__s)->s_dev)

/*
 * Return the interface for this sensor
 */
#define SENSOR_GET_ITF(__s) (&((__s)->s_itf))

#define SENSOR_DATA_CMP_GT(__d, __t, __f) (\
    ((__d->__f##_is_valid) && (__t->__f##_is_valid)) ? (__d->__f > __t->__f) : (0))

#define SENSOR_DATA_CMP_LT(__d, __t, __f) (\
    ((__d->__f##_is_valid) && (__t->__f##_is_valid)) ? (__d->__f < __t->__f) : (0))

struct sensor {
    /* The OS device this sensor inherits from, this is typically a sensor
     * specific driver.
     */
    struct os_dev *s_dev;

    /* The lock for this sensor object */
    struct os_mutex s_lock;


    /* A bit mask describing the types of sensor objects available from this
     * sensor. If the bit corresponding to the sensor_type_t is set, then this
     * sensor supports that variable.
     */
    sensor_type_t s_types;

    /* Sensor mask */
    sensor_type_t s_mask;
    /**
     * Poll rate in MS for this sensor.
     */
    uint32_t s_poll_rate;

    /* The next time at which we want to poll data from this sensor */
    os_time_t s_next_run;

    /* Sensor driver specific functions, created by the device registering the
     * sensor.
     */
    struct sensor_driver *s_funcs;

    /* Sensor last reading timestamp */
    struct sensor_timestamp s_sts;

    /* Sensor interface structure */
    struct sensor_itf s_itf;

    /* A list of listeners that are registered to receive data off of this
     * sensor
     */
    SLIST_HEAD(, sensor_listener) s_listener_list;

    /* A list of notifiers that are registered to receive events from this
     * sensor
     */
    SLIST_HEAD(, sensor_notifier) s_notifier_list;

    /* A list of sensor thresholds that are registered */
    SLIST_HEAD(, sensor_type_traits) s_type_traits_list;

    /* The next sensor in the global sensor list. */
    SLIST_ENTRY(sensor) s_next;
};

int sensor_init(struct sensor *, struct os_dev *);
int sensor_lock(struct sensor *);
void sensor_unlock(struct sensor *);

/**
 * Register a sensor listener. This allows a calling application to receive
 * callbacks for data from a given sensor object.
 *
 * For more information on the type of callbacks available, see the documentation
 * for the sensor listener structure.
 *
 * @param The sensor to register a listener on
 * @param The listener to register onto the sensor
 *
 * @return 0 on success, non-zero error code on failure.
 */
int sensor_register_listener(struct sensor *, struct sensor_listener *);

/**
 * Un-register a sensor listener. This allows a calling application to clear
 * callbacks for a given sensor object.
 *
 * @param The sensor object
 * @param The listener to remove from the sensor listener list
 *
 * @return 0 on success, non-zero error code on failure.
 */

int sensor_unregister_listener(struct sensor *, struct sensor_listener *);

/**
 * Register a sensor notifier. This allows a calling application to receive
 * callbacks any time a requested event is observed.
 *
 * @param The sensor to register the notifier on
 * @param The notifier to register
 *
 * @return 0 on success, non-zero error code on failure.
 */
int sensor_register_notifier(struct sensor *, struct sensor_notifier *);

/**
 * Un-register a sensor notifier. This allows a calling application to stop
 * receiving callbacks for events on the sensor object.
 *
 * @param The sensor object to un-register the notifier on
 * @param The notifier to remove from the notification list
 *
 * @return 0 on success, non-zero error code on failure.
 */
int sensor_unregister_notifier(struct sensor *, struct sensor_notifier *);

int sensor_read(struct sensor *, sensor_type_t, sensor_data_func_t, void *,
        uint32_t);

/**
 * Set the driver functions for this sensor, along with the type of sensor
 * data available for the given sensor.
 *
 * @param The sensor to set the driver information for
 * @param The types of sensor data available for this sensor
 * @param The driver functions for this sensor
 *
 * @return 0 on success, non-zero error code on failure
 */
static inline int
sensor_set_driver(struct sensor *sensor, sensor_type_t type,
        struct sensor_driver *driver)
{
    sensor->s_funcs = driver;
    sensor->s_types = type;

    return (0);
}

/**
 * Set the sensor driver mask so that the developer who configures the
 * sensor tells the sensor framework which sensor data to send back to
 * the user
 *
 * @param The sensor to set the mask for
 * @param The mask
 */
static inline int
sensor_set_type_mask(struct sensor *sensor, sensor_type_t mask)
{
    sensor->s_mask = mask;

    return (0);
}

/**
 * Check if sensor type is supported by the sensor device
 *
 * @param sensor object
 * @param Type to be checked
 * @return type bitfield, if supported, 0 if not supported
 */
static inline sensor_type_t
sensor_check_type(struct sensor *sensor, sensor_type_t type)
{
    return (sensor->s_types & sensor->s_mask & type);
}

/**
 * Set interface type and number
 *
 * @param The sensor to set the interface for
 * @param The interface type to set
 * @param The interface number to set
 */
static inline int
sensor_set_interface(struct sensor *sensor, struct sensor_itf *s_itf)
{
    memcpy(&sensor->s_itf, s_itf, sizeof(*s_itf));
    return (0);
}

/**
 * Read the configuration for the sensor type "type," and return the
 * configuration into "cfg."
 *
 * @param sensor The sensor to read configuration for
 * @param type The type of sensor configuration to read
 * @param cfg The configuration structure to point to.
 *
 * @return 0 on success, non-zero error code on failure.
 */
static inline int
sensor_get_config(struct sensor *sensor, sensor_type_t type,
        struct sensor_cfg *cfg)
{
    return (sensor->s_funcs->sd_get_config(sensor, type, cfg));
}

/**
 * @{ Sensor Manager API
 */


/* Default number of ticks to wakeup the sensor task.  If sensor
 * polling has been configured more frequently than this, it will
 * be triggered instead.
 */

int sensor_mgr_lock(void);
void sensor_mgr_unlock(void);
int sensor_mgr_register(struct sensor *);
struct os_eventq *sensor_mgr_evq_get(void);


typedef int (*sensor_mgr_compare_func_t)(struct sensor *, void *);

/**
 * The sensor manager contains a list of sensors, this function returns
 * the next sensor in that list, for which compare_func() returns successful
 * (one).  If prev_cursor is provided, the function starts at that point
 * in the sensor list.
 *
 * @warn This function MUST be locked by sensor_mgr_lock/unlock() if the goal is
 * to iterate through sensors (as opposed to just finding one.)  As the
 * "prev_cursor" may be resorted in the sensor list, in between calls.
 *
 * @param The comparison function to use against sensors in the list.
 * @param The argument to provide to that comparison function
 * @param The previous sensor in the sensor manager list, in case of
 *        iteration.  If desire is to find first matching sensor, provide a
 *        NULL value.
 *
 * @return A pointer to the first sensor found from prev_cursor, or
 *         NULL, if none found.
 *
 */
struct sensor *sensor_mgr_find_next(sensor_mgr_compare_func_t, void *,
        struct sensor *);

/**
 * Find the "next" sensor available for a given sensor type.
 *
 * If the sensor parameter, is present find the next entry from that
 * parameter.  Otherwise, find the first matching sensor.
 *
 * @param The type of sensor to search for
 * @param The cursor to search from, or NULL to start from the beginning.
 *
 * @return A pointer to the sensor object matching that sensor type, or NULL if
 *         none found.
 */
struct sensor *sensor_mgr_find_next_bytype(sensor_type_t, struct sensor *);

/**
 * Search the sensor list and find the next sensor that corresponds
 * to a given device name.
 *
 * @param The device name to search for
 * @param The previous sensor found with this device name
 *
 * @return 0 on success, non-zero error code on failure
 */
struct sensor *sensor_mgr_find_next_bydevname(char *, struct sensor *);

/**
 * Check if sensor type matches
 *
 * @param The sensor object
 * @param The type to check
 *
 * @return 1 if matches, 0 if it doesn't match.
 */
int sensor_mgr_match_bytype(struct sensor *, void *);

/**
 * Set the sensor poll rate
 *
 * @param The devname
 * @param The poll rate in milli seconds
 */
int
sensor_set_poll_rate_ms(char *, uint32_t);

/**
 * Set the sensor poll rate multiple based on the device name, sensor type
 *
 * @param The devname
 * @param The sensor type trait
 * @param The multiple of the poll rate
 */
int
sensor_set_n_poll_rate(char *, struct sensor_type_traits *);

/**
 * Transmit OIC trigger
 *
 * @param ptr to the sensor
 * @param ptr to sensor data
 * @param The sensor type
 *
 * @return 0 on sucess, non-zero on failure
 */
int
sensor_oic_tx_trigger(struct sensor *, void *, sensor_type_t);

/**
 * Sensor trigger initialization
 *
 * @param ptr to the sensor sturucture
 * @param sensor type to enable trigger for
 * @param the function to call if the trigger condition is satisfied
 */
void
sensor_trigger_init(struct sensor *, sensor_type_t,
                    sensor_trigger_notify_func_t);

/**
 * Search the sensor type traits list for specific type of sensor
 *
 * @param The sensor type to search for
 * @param Ptr to a sensor
 *
 * @return NULL when no sensor type is found, ptr to sensor_type_traits structure
 * when found
 */
struct sensor_type_traits *
sensor_get_type_traits_bytype(sensor_type_t, struct sensor *);

/**
 * Get the type traits for a sensor
 *
 * @param name of the sensor
 * @param Ptr to sensor types trait struct
 * @param type of sensor
 *
 * @return NULL on failure, sensor struct on success
 */
struct sensor *
sensor_get_type_traits_byname(char *, struct sensor_type_traits **,
                              sensor_type_t);

/**
 * Set the thresholds along with the comparison algo for a sensor
 *
 * @param name of the sensor
 * @param Ptr to sensor type traits containing thresholds
 *
 * @return 0 on success, non-zero on failure
 */
int
sensor_set_thresh(char *, struct sensor_type_traits *);

/**
 * Clears the low threshold for a sensor
 *
 * @param name of the sensor
 * @param sensor type
 *
 * @return 0 on success, non-zero on failure
 */
int
sensor_clear_low_thresh(char *, sensor_type_t);

/**
 * Clears the high threshold for a sensor
 *
 * @param name of the sensor
 * @param sensor type
 *
 * @return 0 on success, non-zero on failure
 */
int
sensor_clear_high_thresh(char *, sensor_type_t);

/**
 * Set the watermark thresholds for a sensor
 *
 * @param name of the sensor
 * @param Ptr to sensor type traits containing thresholds
 *
 * @return 0 on success, non-zero on failure
 */
int
sensor_set_watermark_thresh(char *, struct sensor_type_traits *);

/**
 * Puts a notification event on the sensor manager evq
 *
 * @param notification event context
 */
void
sensor_mgr_put_notify_evt(struct sensor_notify_ev_ctx *);

/**
 * Puts a interrupt event on the sensor manager evq
 *
 * @param interrupt event context
 */
void
sensor_mgr_put_interrupt_evt(struct sensor *);

/**
 * Puts read event on the sensor manager evq
 *
 * @param arg
 */
void
sensor_mgr_put_read_evt(void *);

#if MYNEWT_VAL(SENSOR_CLI)
char*
sensor_ftostr(float, char *, int);
#endif

#if MYNEWT_VAL(SENSOR_OIC)
void sensor_oic_init(void);
#endif

/**
 * }@
 */



/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_H__ */
