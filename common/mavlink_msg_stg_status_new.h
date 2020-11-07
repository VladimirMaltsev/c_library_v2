#pragma once
// MESSAGE STG_STATUS_NEW PACKING

#define MAVLINK_MSG_ID_STG_STATUS_NEW 20001


typedef struct __mavlink_stg_status_new_t {
 uint16_t voltage_battery; /*< [mV] Battery voltage.*/
 uint16_t voltage_generator; /*< [mV] Generator voltage.*/
 uint16_t current_battery; /*< [mA] Battery current.*/
 uint16_t power_load; /*< [W] Load power.*/
 uint16_t current_charge; /*< [mA] Charger current. 0 - charging stopped.*/
 int16_t rpm_cranckshaft; /*< [rpm] Cranckshaft rotations per minute.*/
 uint16_t uptime; /*< [s] Uptime.*/
 uint8_t fuel_level; /*< [%] Fuel level in persent.*/
 uint8_t bridge_temperature; /*< [degC] Bridge temperature.*/
 uint8_t engine_temperature; /*< [degC] Engine temperature.*/
 uint8_t motor_state; /*<  Motor state.*/
 uint8_t stg_errors_bitmask; /*<  Errors bitmask.*/
} mavlink_stg_status_new_t;

#define MAVLINK_MSG_ID_STG_STATUS_NEW_LEN 19
#define MAVLINK_MSG_ID_STG_STATUS_NEW_MIN_LEN 19
#define MAVLINK_MSG_ID_20001_LEN 19
#define MAVLINK_MSG_ID_20001_MIN_LEN 19

#define MAVLINK_MSG_ID_STG_STATUS_NEW_CRC 85
#define MAVLINK_MSG_ID_20001_CRC 85



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_STG_STATUS_NEW { \
    20001, \
    "STG_STATUS_NEW", \
    12, \
    {  { "voltage_battery", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_stg_status_new_t, voltage_battery) }, \
         { "voltage_generator", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_stg_status_new_t, voltage_generator) }, \
         { "current_battery", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_stg_status_new_t, current_battery) }, \
         { "power_load", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_stg_status_new_t, power_load) }, \
         { "current_charge", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_stg_status_new_t, current_charge) }, \
         { "rpm_cranckshaft", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_stg_status_new_t, rpm_cranckshaft) }, \
         { "uptime", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_stg_status_new_t, uptime) }, \
         { "fuel_level", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_stg_status_new_t, fuel_level) }, \
         { "bridge_temperature", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_stg_status_new_t, bridge_temperature) }, \
         { "engine_temperature", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_stg_status_new_t, engine_temperature) }, \
         { "motor_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_stg_status_new_t, motor_state) }, \
         { "stg_errors_bitmask", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_stg_status_new_t, stg_errors_bitmask) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_STG_STATUS_NEW { \
    "STG_STATUS_NEW", \
    12, \
    {  { "voltage_battery", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_stg_status_new_t, voltage_battery) }, \
         { "voltage_generator", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_stg_status_new_t, voltage_generator) }, \
         { "current_battery", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_stg_status_new_t, current_battery) }, \
         { "power_load", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_stg_status_new_t, power_load) }, \
         { "current_charge", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_stg_status_new_t, current_charge) }, \
         { "rpm_cranckshaft", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_stg_status_new_t, rpm_cranckshaft) }, \
         { "uptime", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_stg_status_new_t, uptime) }, \
         { "fuel_level", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_stg_status_new_t, fuel_level) }, \
         { "bridge_temperature", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_stg_status_new_t, bridge_temperature) }, \
         { "engine_temperature", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_stg_status_new_t, engine_temperature) }, \
         { "motor_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_stg_status_new_t, motor_state) }, \
         { "stg_errors_bitmask", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_stg_status_new_t, stg_errors_bitmask) }, \
         } \
}
#endif

/**
 * @brief Pack a stg_status_new message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param voltage_battery [mV] Battery voltage.
 * @param voltage_generator [mV] Generator voltage.
 * @param current_battery [mA] Battery current.
 * @param power_load [W] Load power.
 * @param current_charge [mA] Charger current. 0 - charging stopped.
 * @param rpm_cranckshaft [rpm] Cranckshaft rotations per minute.
 * @param uptime [s] Uptime.
 * @param fuel_level [%] Fuel level in persent.
 * @param bridge_temperature [degC] Bridge temperature.
 * @param engine_temperature [degC] Engine temperature.
 * @param motor_state  Motor state.
 * @param stg_errors_bitmask  Errors bitmask.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_stg_status_new_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t voltage_battery, uint16_t voltage_generator, uint16_t current_battery, uint16_t power_load, uint16_t current_charge, int16_t rpm_cranckshaft, uint16_t uptime, uint8_t fuel_level, uint8_t bridge_temperature, uint8_t engine_temperature, uint8_t motor_state, uint8_t stg_errors_bitmask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STG_STATUS_NEW_LEN];
    _mav_put_uint16_t(buf, 0, voltage_battery);
    _mav_put_uint16_t(buf, 2, voltage_generator);
    _mav_put_uint16_t(buf, 4, current_battery);
    _mav_put_uint16_t(buf, 6, power_load);
    _mav_put_uint16_t(buf, 8, current_charge);
    _mav_put_int16_t(buf, 10, rpm_cranckshaft);
    _mav_put_uint16_t(buf, 12, uptime);
    _mav_put_uint8_t(buf, 14, fuel_level);
    _mav_put_uint8_t(buf, 15, bridge_temperature);
    _mav_put_uint8_t(buf, 16, engine_temperature);
    _mav_put_uint8_t(buf, 17, motor_state);
    _mav_put_uint8_t(buf, 18, stg_errors_bitmask);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STG_STATUS_NEW_LEN);
#else
    mavlink_stg_status_new_t packet;
    packet.voltage_battery = voltage_battery;
    packet.voltage_generator = voltage_generator;
    packet.current_battery = current_battery;
    packet.power_load = power_load;
    packet.current_charge = current_charge;
    packet.rpm_cranckshaft = rpm_cranckshaft;
    packet.uptime = uptime;
    packet.fuel_level = fuel_level;
    packet.bridge_temperature = bridge_temperature;
    packet.engine_temperature = engine_temperature;
    packet.motor_state = motor_state;
    packet.stg_errors_bitmask = stg_errors_bitmask;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STG_STATUS_NEW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STG_STATUS_NEW;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_STG_STATUS_NEW_MIN_LEN, MAVLINK_MSG_ID_STG_STATUS_NEW_LEN, MAVLINK_MSG_ID_STG_STATUS_NEW_CRC);
}

/**
 * @brief Pack a stg_status_new message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param voltage_battery [mV] Battery voltage.
 * @param voltage_generator [mV] Generator voltage.
 * @param current_battery [mA] Battery current.
 * @param power_load [W] Load power.
 * @param current_charge [mA] Charger current. 0 - charging stopped.
 * @param rpm_cranckshaft [rpm] Cranckshaft rotations per minute.
 * @param uptime [s] Uptime.
 * @param fuel_level [%] Fuel level in persent.
 * @param bridge_temperature [degC] Bridge temperature.
 * @param engine_temperature [degC] Engine temperature.
 * @param motor_state  Motor state.
 * @param stg_errors_bitmask  Errors bitmask.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_stg_status_new_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t voltage_battery,uint16_t voltage_generator,uint16_t current_battery,uint16_t power_load,uint16_t current_charge,int16_t rpm_cranckshaft,uint16_t uptime,uint8_t fuel_level,uint8_t bridge_temperature,uint8_t engine_temperature,uint8_t motor_state,uint8_t stg_errors_bitmask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STG_STATUS_NEW_LEN];
    _mav_put_uint16_t(buf, 0, voltage_battery);
    _mav_put_uint16_t(buf, 2, voltage_generator);
    _mav_put_uint16_t(buf, 4, current_battery);
    _mav_put_uint16_t(buf, 6, power_load);
    _mav_put_uint16_t(buf, 8, current_charge);
    _mav_put_int16_t(buf, 10, rpm_cranckshaft);
    _mav_put_uint16_t(buf, 12, uptime);
    _mav_put_uint8_t(buf, 14, fuel_level);
    _mav_put_uint8_t(buf, 15, bridge_temperature);
    _mav_put_uint8_t(buf, 16, engine_temperature);
    _mav_put_uint8_t(buf, 17, motor_state);
    _mav_put_uint8_t(buf, 18, stg_errors_bitmask);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STG_STATUS_NEW_LEN);
#else
    mavlink_stg_status_new_t packet;
    packet.voltage_battery = voltage_battery;
    packet.voltage_generator = voltage_generator;
    packet.current_battery = current_battery;
    packet.power_load = power_load;
    packet.current_charge = current_charge;
    packet.rpm_cranckshaft = rpm_cranckshaft;
    packet.uptime = uptime;
    packet.fuel_level = fuel_level;
    packet.bridge_temperature = bridge_temperature;
    packet.engine_temperature = engine_temperature;
    packet.motor_state = motor_state;
    packet.stg_errors_bitmask = stg_errors_bitmask;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STG_STATUS_NEW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STG_STATUS_NEW;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_STG_STATUS_NEW_MIN_LEN, MAVLINK_MSG_ID_STG_STATUS_NEW_LEN, MAVLINK_MSG_ID_STG_STATUS_NEW_CRC);
}

/**
 * @brief Encode a stg_status_new struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param stg_status_new C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_stg_status_new_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_stg_status_new_t* stg_status_new)
{
    return mavlink_msg_stg_status_new_pack(system_id, component_id, msg, stg_status_new->voltage_battery, stg_status_new->voltage_generator, stg_status_new->current_battery, stg_status_new->power_load, stg_status_new->current_charge, stg_status_new->rpm_cranckshaft, stg_status_new->uptime, stg_status_new->fuel_level, stg_status_new->bridge_temperature, stg_status_new->engine_temperature, stg_status_new->motor_state, stg_status_new->stg_errors_bitmask);
}

/**
 * @brief Encode a stg_status_new struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param stg_status_new C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_stg_status_new_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_stg_status_new_t* stg_status_new)
{
    return mavlink_msg_stg_status_new_pack_chan(system_id, component_id, chan, msg, stg_status_new->voltage_battery, stg_status_new->voltage_generator, stg_status_new->current_battery, stg_status_new->power_load, stg_status_new->current_charge, stg_status_new->rpm_cranckshaft, stg_status_new->uptime, stg_status_new->fuel_level, stg_status_new->bridge_temperature, stg_status_new->engine_temperature, stg_status_new->motor_state, stg_status_new->stg_errors_bitmask);
}

/**
 * @brief Send a stg_status_new message
 * @param chan MAVLink channel to send the message
 *
 * @param voltage_battery [mV] Battery voltage.
 * @param voltage_generator [mV] Generator voltage.
 * @param current_battery [mA] Battery current.
 * @param power_load [W] Load power.
 * @param current_charge [mA] Charger current. 0 - charging stopped.
 * @param rpm_cranckshaft [rpm] Cranckshaft rotations per minute.
 * @param uptime [s] Uptime.
 * @param fuel_level [%] Fuel level in persent.
 * @param bridge_temperature [degC] Bridge temperature.
 * @param engine_temperature [degC] Engine temperature.
 * @param motor_state  Motor state.
 * @param stg_errors_bitmask  Errors bitmask.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_stg_status_new_send(mavlink_channel_t chan, uint16_t voltage_battery, uint16_t voltage_generator, uint16_t current_battery, uint16_t power_load, uint16_t current_charge, int16_t rpm_cranckshaft, uint16_t uptime, uint8_t fuel_level, uint8_t bridge_temperature, uint8_t engine_temperature, uint8_t motor_state, uint8_t stg_errors_bitmask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STG_STATUS_NEW_LEN];
    _mav_put_uint16_t(buf, 0, voltage_battery);
    _mav_put_uint16_t(buf, 2, voltage_generator);
    _mav_put_uint16_t(buf, 4, current_battery);
    _mav_put_uint16_t(buf, 6, power_load);
    _mav_put_uint16_t(buf, 8, current_charge);
    _mav_put_int16_t(buf, 10, rpm_cranckshaft);
    _mav_put_uint16_t(buf, 12, uptime);
    _mav_put_uint8_t(buf, 14, fuel_level);
    _mav_put_uint8_t(buf, 15, bridge_temperature);
    _mav_put_uint8_t(buf, 16, engine_temperature);
    _mav_put_uint8_t(buf, 17, motor_state);
    _mav_put_uint8_t(buf, 18, stg_errors_bitmask);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STG_STATUS_NEW, buf, MAVLINK_MSG_ID_STG_STATUS_NEW_MIN_LEN, MAVLINK_MSG_ID_STG_STATUS_NEW_LEN, MAVLINK_MSG_ID_STG_STATUS_NEW_CRC);
#else
    mavlink_stg_status_new_t packet;
    packet.voltage_battery = voltage_battery;
    packet.voltage_generator = voltage_generator;
    packet.current_battery = current_battery;
    packet.power_load = power_load;
    packet.current_charge = current_charge;
    packet.rpm_cranckshaft = rpm_cranckshaft;
    packet.uptime = uptime;
    packet.fuel_level = fuel_level;
    packet.bridge_temperature = bridge_temperature;
    packet.engine_temperature = engine_temperature;
    packet.motor_state = motor_state;
    packet.stg_errors_bitmask = stg_errors_bitmask;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STG_STATUS_NEW, (const char *)&packet, MAVLINK_MSG_ID_STG_STATUS_NEW_MIN_LEN, MAVLINK_MSG_ID_STG_STATUS_NEW_LEN, MAVLINK_MSG_ID_STG_STATUS_NEW_CRC);
#endif
}

/**
 * @brief Send a stg_status_new message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_stg_status_new_send_struct(mavlink_channel_t chan, const mavlink_stg_status_new_t* stg_status_new)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_stg_status_new_send(chan, stg_status_new->voltage_battery, stg_status_new->voltage_generator, stg_status_new->current_battery, stg_status_new->power_load, stg_status_new->current_charge, stg_status_new->rpm_cranckshaft, stg_status_new->uptime, stg_status_new->fuel_level, stg_status_new->bridge_temperature, stg_status_new->engine_temperature, stg_status_new->motor_state, stg_status_new->stg_errors_bitmask);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STG_STATUS_NEW, (const char *)stg_status_new, MAVLINK_MSG_ID_STG_STATUS_NEW_MIN_LEN, MAVLINK_MSG_ID_STG_STATUS_NEW_LEN, MAVLINK_MSG_ID_STG_STATUS_NEW_CRC);
#endif
}

#if MAVLINK_MSG_ID_STG_STATUS_NEW_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_stg_status_new_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t voltage_battery, uint16_t voltage_generator, uint16_t current_battery, uint16_t power_load, uint16_t current_charge, int16_t rpm_cranckshaft, uint16_t uptime, uint8_t fuel_level, uint8_t bridge_temperature, uint8_t engine_temperature, uint8_t motor_state, uint8_t stg_errors_bitmask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, voltage_battery);
    _mav_put_uint16_t(buf, 2, voltage_generator);
    _mav_put_uint16_t(buf, 4, current_battery);
    _mav_put_uint16_t(buf, 6, power_load);
    _mav_put_uint16_t(buf, 8, current_charge);
    _mav_put_int16_t(buf, 10, rpm_cranckshaft);
    _mav_put_uint16_t(buf, 12, uptime);
    _mav_put_uint8_t(buf, 14, fuel_level);
    _mav_put_uint8_t(buf, 15, bridge_temperature);
    _mav_put_uint8_t(buf, 16, engine_temperature);
    _mav_put_uint8_t(buf, 17, motor_state);
    _mav_put_uint8_t(buf, 18, stg_errors_bitmask);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STG_STATUS_NEW, buf, MAVLINK_MSG_ID_STG_STATUS_NEW_MIN_LEN, MAVLINK_MSG_ID_STG_STATUS_NEW_LEN, MAVLINK_MSG_ID_STG_STATUS_NEW_CRC);
#else
    mavlink_stg_status_new_t *packet = (mavlink_stg_status_new_t *)msgbuf;
    packet->voltage_battery = voltage_battery;
    packet->voltage_generator = voltage_generator;
    packet->current_battery = current_battery;
    packet->power_load = power_load;
    packet->current_charge = current_charge;
    packet->rpm_cranckshaft = rpm_cranckshaft;
    packet->uptime = uptime;
    packet->fuel_level = fuel_level;
    packet->bridge_temperature = bridge_temperature;
    packet->engine_temperature = engine_temperature;
    packet->motor_state = motor_state;
    packet->stg_errors_bitmask = stg_errors_bitmask;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STG_STATUS_NEW, (const char *)packet, MAVLINK_MSG_ID_STG_STATUS_NEW_MIN_LEN, MAVLINK_MSG_ID_STG_STATUS_NEW_LEN, MAVLINK_MSG_ID_STG_STATUS_NEW_CRC);
#endif
}
#endif

#endif

// MESSAGE STG_STATUS_NEW UNPACKING


/**
 * @brief Get field voltage_battery from stg_status_new message
 *
 * @return [mV] Battery voltage.
 */
static inline uint16_t mavlink_msg_stg_status_new_get_voltage_battery(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field voltage_generator from stg_status_new message
 *
 * @return [mV] Generator voltage.
 */
static inline uint16_t mavlink_msg_stg_status_new_get_voltage_generator(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field current_battery from stg_status_new message
 *
 * @return [mA] Battery current.
 */
static inline uint16_t mavlink_msg_stg_status_new_get_current_battery(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field power_load from stg_status_new message
 *
 * @return [W] Load power.
 */
static inline uint16_t mavlink_msg_stg_status_new_get_power_load(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field current_charge from stg_status_new message
 *
 * @return [mA] Charger current. 0 - charging stopped.
 */
static inline uint16_t mavlink_msg_stg_status_new_get_current_charge(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field rpm_cranckshaft from stg_status_new message
 *
 * @return [rpm] Cranckshaft rotations per minute.
 */
static inline int16_t mavlink_msg_stg_status_new_get_rpm_cranckshaft(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field uptime from stg_status_new message
 *
 * @return [s] Uptime.
 */
static inline uint16_t mavlink_msg_stg_status_new_get_uptime(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field fuel_level from stg_status_new message
 *
 * @return [%] Fuel level in persent.
 */
static inline uint8_t mavlink_msg_stg_status_new_get_fuel_level(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field bridge_temperature from stg_status_new message
 *
 * @return [degC] Bridge temperature.
 */
static inline uint8_t mavlink_msg_stg_status_new_get_bridge_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Get field engine_temperature from stg_status_new message
 *
 * @return [degC] Engine temperature.
 */
static inline uint8_t mavlink_msg_stg_status_new_get_engine_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field motor_state from stg_status_new message
 *
 * @return  Motor state.
 */
static inline uint8_t mavlink_msg_stg_status_new_get_motor_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field stg_errors_bitmask from stg_status_new message
 *
 * @return  Errors bitmask.
 */
static inline uint8_t mavlink_msg_stg_status_new_get_stg_errors_bitmask(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Decode a stg_status_new message into a struct
 *
 * @param msg The message to decode
 * @param stg_status_new C-struct to decode the message contents into
 */
static inline void mavlink_msg_stg_status_new_decode(const mavlink_message_t* msg, mavlink_stg_status_new_t* stg_status_new)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    stg_status_new->voltage_battery = mavlink_msg_stg_status_new_get_voltage_battery(msg);
    stg_status_new->voltage_generator = mavlink_msg_stg_status_new_get_voltage_generator(msg);
    stg_status_new->current_battery = mavlink_msg_stg_status_new_get_current_battery(msg);
    stg_status_new->power_load = mavlink_msg_stg_status_new_get_power_load(msg);
    stg_status_new->current_charge = mavlink_msg_stg_status_new_get_current_charge(msg);
    stg_status_new->rpm_cranckshaft = mavlink_msg_stg_status_new_get_rpm_cranckshaft(msg);
    stg_status_new->uptime = mavlink_msg_stg_status_new_get_uptime(msg);
    stg_status_new->fuel_level = mavlink_msg_stg_status_new_get_fuel_level(msg);
    stg_status_new->bridge_temperature = mavlink_msg_stg_status_new_get_bridge_temperature(msg);
    stg_status_new->engine_temperature = mavlink_msg_stg_status_new_get_engine_temperature(msg);
    stg_status_new->motor_state = mavlink_msg_stg_status_new_get_motor_state(msg);
    stg_status_new->stg_errors_bitmask = mavlink_msg_stg_status_new_get_stg_errors_bitmask(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_STG_STATUS_NEW_LEN? msg->len : MAVLINK_MSG_ID_STG_STATUS_NEW_LEN;
        memset(stg_status_new, 0, MAVLINK_MSG_ID_STG_STATUS_NEW_LEN);
    memcpy(stg_status_new, _MAV_PAYLOAD(msg), len);
#endif
}
