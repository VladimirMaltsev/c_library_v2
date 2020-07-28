#pragma once
// MESSAGE ADC_REPORT PACKING

#define MAVLINK_MSG_ID_ADC_REPORT 20002


typedef struct __mavlink_adc_report_t {
 float channel_value[12]; /*<   ADC channel value in volt, valid if channel ID is positive.*/
 uint16_t channel_id[12]; /*<  ADC channel IDs, negative for non-existent.*/
} mavlink_adc_report_t;

#define MAVLINK_MSG_ID_ADC_REPORT_LEN 72
#define MAVLINK_MSG_ID_ADC_REPORT_MIN_LEN 72
#define MAVLINK_MSG_ID_20002_LEN 72
#define MAVLINK_MSG_ID_20002_MIN_LEN 72

#define MAVLINK_MSG_ID_ADC_REPORT_CRC 176
#define MAVLINK_MSG_ID_20002_CRC 176

#define MAVLINK_MSG_ADC_REPORT_FIELD_CHANNEL_VALUE_LEN 12
#define MAVLINK_MSG_ADC_REPORT_FIELD_CHANNEL_ID_LEN 12

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ADC_REPORT { \
    20002, \
    "ADC_REPORT", \
    2, \
    {  { "channel_id", NULL, MAVLINK_TYPE_UINT16_T, 12, 48, offsetof(mavlink_adc_report_t, channel_id) }, \
         { "channel_value", NULL, MAVLINK_TYPE_FLOAT, 12, 0, offsetof(mavlink_adc_report_t, channel_value) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ADC_REPORT { \
    "ADC_REPORT", \
    2, \
    {  { "channel_id", NULL, MAVLINK_TYPE_UINT16_T, 12, 48, offsetof(mavlink_adc_report_t, channel_id) }, \
         { "channel_value", NULL, MAVLINK_TYPE_FLOAT, 12, 0, offsetof(mavlink_adc_report_t, channel_value) }, \
         } \
}
#endif

/**
 * @brief Pack a adc_report message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param channel_id  ADC channel IDs, negative for non-existent.
 * @param channel_value   ADC channel value in volt, valid if channel ID is positive.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adc_report_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const uint16_t *channel_id, const float *channel_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADC_REPORT_LEN];

    _mav_put_float_array(buf, 0, channel_value, 12);
    _mav_put_uint16_t_array(buf, 48, channel_id, 12);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADC_REPORT_LEN);
#else
    mavlink_adc_report_t packet;

    mav_array_memcpy(packet.channel_value, channel_value, sizeof(float)*12);
    mav_array_memcpy(packet.channel_id, channel_id, sizeof(uint16_t)*12);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADC_REPORT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ADC_REPORT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ADC_REPORT_MIN_LEN, MAVLINK_MSG_ID_ADC_REPORT_LEN, MAVLINK_MSG_ID_ADC_REPORT_CRC);
}

/**
 * @brief Pack a adc_report message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param channel_id  ADC channel IDs, negative for non-existent.
 * @param channel_value   ADC channel value in volt, valid if channel ID is positive.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adc_report_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const uint16_t *channel_id,const float *channel_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADC_REPORT_LEN];

    _mav_put_float_array(buf, 0, channel_value, 12);
    _mav_put_uint16_t_array(buf, 48, channel_id, 12);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADC_REPORT_LEN);
#else
    mavlink_adc_report_t packet;

    mav_array_memcpy(packet.channel_value, channel_value, sizeof(float)*12);
    mav_array_memcpy(packet.channel_id, channel_id, sizeof(uint16_t)*12);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADC_REPORT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ADC_REPORT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ADC_REPORT_MIN_LEN, MAVLINK_MSG_ID_ADC_REPORT_LEN, MAVLINK_MSG_ID_ADC_REPORT_CRC);
}

/**
 * @brief Encode a adc_report struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param adc_report C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adc_report_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_adc_report_t* adc_report)
{
    return mavlink_msg_adc_report_pack(system_id, component_id, msg, adc_report->channel_id, adc_report->channel_value);
}

/**
 * @brief Encode a adc_report struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param adc_report C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adc_report_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_adc_report_t* adc_report)
{
    return mavlink_msg_adc_report_pack_chan(system_id, component_id, chan, msg, adc_report->channel_id, adc_report->channel_value);
}

/**
 * @brief Send a adc_report message
 * @param chan MAVLink channel to send the message
 *
 * @param channel_id  ADC channel IDs, negative for non-existent.
 * @param channel_value   ADC channel value in volt, valid if channel ID is positive.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_adc_report_send(mavlink_channel_t chan, const uint16_t *channel_id, const float *channel_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADC_REPORT_LEN];

    _mav_put_float_array(buf, 0, channel_value, 12);
    _mav_put_uint16_t_array(buf, 48, channel_id, 12);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADC_REPORT, buf, MAVLINK_MSG_ID_ADC_REPORT_MIN_LEN, MAVLINK_MSG_ID_ADC_REPORT_LEN, MAVLINK_MSG_ID_ADC_REPORT_CRC);
#else
    mavlink_adc_report_t packet;

    mav_array_memcpy(packet.channel_value, channel_value, sizeof(float)*12);
    mav_array_memcpy(packet.channel_id, channel_id, sizeof(uint16_t)*12);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADC_REPORT, (const char *)&packet, MAVLINK_MSG_ID_ADC_REPORT_MIN_LEN, MAVLINK_MSG_ID_ADC_REPORT_LEN, MAVLINK_MSG_ID_ADC_REPORT_CRC);
#endif
}

/**
 * @brief Send a adc_report message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_adc_report_send_struct(mavlink_channel_t chan, const mavlink_adc_report_t* adc_report)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_adc_report_send(chan, adc_report->channel_id, adc_report->channel_value);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADC_REPORT, (const char *)adc_report, MAVLINK_MSG_ID_ADC_REPORT_MIN_LEN, MAVLINK_MSG_ID_ADC_REPORT_LEN, MAVLINK_MSG_ID_ADC_REPORT_CRC);
#endif
}

#if MAVLINK_MSG_ID_ADC_REPORT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_adc_report_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const uint16_t *channel_id, const float *channel_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_float_array(buf, 0, channel_value, 12);
    _mav_put_uint16_t_array(buf, 48, channel_id, 12);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADC_REPORT, buf, MAVLINK_MSG_ID_ADC_REPORT_MIN_LEN, MAVLINK_MSG_ID_ADC_REPORT_LEN, MAVLINK_MSG_ID_ADC_REPORT_CRC);
#else
    mavlink_adc_report_t *packet = (mavlink_adc_report_t *)msgbuf;

    mav_array_memcpy(packet->channel_value, channel_value, sizeof(float)*12);
    mav_array_memcpy(packet->channel_id, channel_id, sizeof(uint16_t)*12);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADC_REPORT, (const char *)packet, MAVLINK_MSG_ID_ADC_REPORT_MIN_LEN, MAVLINK_MSG_ID_ADC_REPORT_LEN, MAVLINK_MSG_ID_ADC_REPORT_CRC);
#endif
}
#endif

#endif

// MESSAGE ADC_REPORT UNPACKING


/**
 * @brief Get field channel_id from adc_report message
 *
 * @return  ADC channel IDs, negative for non-existent.
 */
static inline uint16_t mavlink_msg_adc_report_get_channel_id(const mavlink_message_t* msg, uint16_t *channel_id)
{
    return _MAV_RETURN_uint16_t_array(msg, channel_id, 12,  48);
}

/**
 * @brief Get field channel_value from adc_report message
 *
 * @return   ADC channel value in volt, valid if channel ID is positive.
 */
static inline uint16_t mavlink_msg_adc_report_get_channel_value(const mavlink_message_t* msg, float *channel_value)
{
    return _MAV_RETURN_float_array(msg, channel_value, 12,  0);
}

/**
 * @brief Decode a adc_report message into a struct
 *
 * @param msg The message to decode
 * @param adc_report C-struct to decode the message contents into
 */
static inline void mavlink_msg_adc_report_decode(const mavlink_message_t* msg, mavlink_adc_report_t* adc_report)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_adc_report_get_channel_value(msg, adc_report->channel_value);
    mavlink_msg_adc_report_get_channel_id(msg, adc_report->channel_id);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ADC_REPORT_LEN? msg->len : MAVLINK_MSG_ID_ADC_REPORT_LEN;
        memset(adc_report, 0, MAVLINK_MSG_ID_ADC_REPORT_LEN);
    memcpy(adc_report, _MAV_PAYLOAD(msg), len);
#endif
}
