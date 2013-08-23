// MESSAGE SLUGS_RTB PACKING

#define MAVLINK_MSG_ID_SLUGS_RTB 187

typedef struct __mavlink_slugs_rtb_t
{
 uint8_t target; ///< The system ordered to RTB
 uint8_t rtb; ///< Order SLUGS to: 0: Stop RTB and resume flight; 1: RTB
 uint8_t track_mobile; ///< Order SLUGS to: 0: RTB to GS Location; 1: Track mobile 
} mavlink_slugs_rtb_t;

#define MAVLINK_MSG_ID_SLUGS_RTB_LEN 3
#define MAVLINK_MSG_ID_187_LEN 3

#define MAVLINK_MSG_ID_SLUGS_RTB_CRC 250
#define MAVLINK_MSG_ID_187_CRC 250



#define MAVLINK_MESSAGE_INFO_SLUGS_RTB { \
	"SLUGS_RTB", \
	3, \
	{  { "target", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_slugs_rtb_t, target) }, \
         { "rtb", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_slugs_rtb_t, rtb) }, \
         { "track_mobile", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_slugs_rtb_t, track_mobile) }, \
         } \
}


/**
 * @brief Pack a slugs_rtb message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target The system ordered to RTB
 * @param rtb Order SLUGS to: 0: Stop RTB and resume flight; 1: RTB
 * @param track_mobile Order SLUGS to: 0: RTB to GS Location; 1: Track mobile 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_slugs_rtb_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target, uint8_t rtb, uint8_t track_mobile)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SLUGS_RTB_LEN];
	_mav_put_uint8_t(buf, 0, target);
	_mav_put_uint8_t(buf, 1, rtb);
	_mav_put_uint8_t(buf, 2, track_mobile);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SLUGS_RTB_LEN);
#else
	mavlink_slugs_rtb_t packet;
	packet.target = target;
	packet.rtb = rtb;
	packet.track_mobile = track_mobile;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SLUGS_RTB_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SLUGS_RTB;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SLUGS_RTB_LEN, MAVLINK_MSG_ID_SLUGS_RTB_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SLUGS_RTB_LEN);
#endif
}

/**
 * @brief Pack a slugs_rtb message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target The system ordered to RTB
 * @param rtb Order SLUGS to: 0: Stop RTB and resume flight; 1: RTB
 * @param track_mobile Order SLUGS to: 0: RTB to GS Location; 1: Track mobile 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_slugs_rtb_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target,uint8_t rtb,uint8_t track_mobile)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SLUGS_RTB_LEN];
	_mav_put_uint8_t(buf, 0, target);
	_mav_put_uint8_t(buf, 1, rtb);
	_mav_put_uint8_t(buf, 2, track_mobile);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SLUGS_RTB_LEN);
#else
	mavlink_slugs_rtb_t packet;
	packet.target = target;
	packet.rtb = rtb;
	packet.track_mobile = track_mobile;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SLUGS_RTB_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SLUGS_RTB;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SLUGS_RTB_LEN, MAVLINK_MSG_ID_SLUGS_RTB_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SLUGS_RTB_LEN);
#endif
}

/**
 * @brief Encode a slugs_rtb struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param slugs_rtb C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_slugs_rtb_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_slugs_rtb_t* slugs_rtb)
{
	return mavlink_msg_slugs_rtb_pack(system_id, component_id, msg, slugs_rtb->target, slugs_rtb->rtb, slugs_rtb->track_mobile);
}

/**
 * @brief Send a slugs_rtb message
 * @param chan MAVLink channel to send the message
 *
 * @param target The system ordered to RTB
 * @param rtb Order SLUGS to: 0: Stop RTB and resume flight; 1: RTB
 * @param track_mobile Order SLUGS to: 0: RTB to GS Location; 1: Track mobile 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_slugs_rtb_send(mavlink_channel_t chan, uint8_t target, uint8_t rtb, uint8_t track_mobile)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SLUGS_RTB_LEN];
	_mav_put_uint8_t(buf, 0, target);
	_mav_put_uint8_t(buf, 1, rtb);
	_mav_put_uint8_t(buf, 2, track_mobile);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLUGS_RTB, buf, MAVLINK_MSG_ID_SLUGS_RTB_LEN, MAVLINK_MSG_ID_SLUGS_RTB_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLUGS_RTB, buf, MAVLINK_MSG_ID_SLUGS_RTB_LEN);
#endif
#else
	mavlink_slugs_rtb_t packet;
	packet.target = target;
	packet.rtb = rtb;
	packet.track_mobile = track_mobile;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLUGS_RTB, (const char *)&packet, MAVLINK_MSG_ID_SLUGS_RTB_LEN, MAVLINK_MSG_ID_SLUGS_RTB_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLUGS_RTB, (const char *)&packet, MAVLINK_MSG_ID_SLUGS_RTB_LEN);
#endif
#endif
}

#endif

// MESSAGE SLUGS_RTB UNPACKING


/**
 * @brief Get field target from slugs_rtb message
 *
 * @return The system ordered to RTB
 */
static inline uint8_t mavlink_msg_slugs_rtb_get_target(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field rtb from slugs_rtb message
 *
 * @return Order SLUGS to: 0: Stop RTB and resume flight; 1: RTB
 */
static inline uint8_t mavlink_msg_slugs_rtb_get_rtb(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field track_mobile from slugs_rtb message
 *
 * @return Order SLUGS to: 0: RTB to GS Location; 1: Track mobile 
 */
static inline uint8_t mavlink_msg_slugs_rtb_get_track_mobile(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a slugs_rtb message into a struct
 *
 * @param msg The message to decode
 * @param slugs_rtb C-struct to decode the message contents into
 */
static inline void mavlink_msg_slugs_rtb_decode(const mavlink_message_t* msg, mavlink_slugs_rtb_t* slugs_rtb)
{
#if MAVLINK_NEED_BYTE_SWAP
	slugs_rtb->target = mavlink_msg_slugs_rtb_get_target(msg);
	slugs_rtb->rtb = mavlink_msg_slugs_rtb_get_rtb(msg);
	slugs_rtb->track_mobile = mavlink_msg_slugs_rtb_get_track_mobile(msg);
#else
	memcpy(slugs_rtb, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SLUGS_RTB_LEN);
#endif
}
