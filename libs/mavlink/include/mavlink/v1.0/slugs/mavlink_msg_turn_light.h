// MESSAGE TURN_LIGHT PACKING

#define MAVLINK_MSG_ID_TURN_LIGHT 190

typedef struct __mavlink_turn_light_t
{
 uint8_t target; ///< The system ordered to turn on lights
 uint8_t type; ///< Type lights 0: Visible; 1: Infrared
 uint8_t turn; ///< Order turn on lights 1: Turn on; 0: Turn off
} mavlink_turn_light_t;

#define MAVLINK_MSG_ID_TURN_LIGHT_LEN 3
#define MAVLINK_MSG_ID_190_LEN 3

#define MAVLINK_MSG_ID_TURN_LIGHT_CRC 229
#define MAVLINK_MSG_ID_190_CRC 229



#define MAVLINK_MESSAGE_INFO_TURN_LIGHT { \
	"TURN_LIGHT", \
	3, \
	{  { "target", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_turn_light_t, target) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_turn_light_t, type) }, \
         { "turn", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_turn_light_t, turn) }, \
         } \
}


/**
 * @brief Pack a turn_light message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target The system ordered to turn on lights
 * @param type Type lights 0: Visible; 1: Infrared
 * @param turn Order turn on lights 1: Turn on; 0: Turn off
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_turn_light_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target, uint8_t type, uint8_t turn)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TURN_LIGHT_LEN];
	_mav_put_uint8_t(buf, 0, target);
	_mav_put_uint8_t(buf, 1, type);
	_mav_put_uint8_t(buf, 2, turn);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TURN_LIGHT_LEN);
#else
	mavlink_turn_light_t packet;
	packet.target = target;
	packet.type = type;
	packet.turn = turn;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TURN_LIGHT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TURN_LIGHT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TURN_LIGHT_LEN, MAVLINK_MSG_ID_TURN_LIGHT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TURN_LIGHT_LEN);
#endif
}

/**
 * @brief Pack a turn_light message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target The system ordered to turn on lights
 * @param type Type lights 0: Visible; 1: Infrared
 * @param turn Order turn on lights 1: Turn on; 0: Turn off
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_turn_light_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target,uint8_t type,uint8_t turn)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TURN_LIGHT_LEN];
	_mav_put_uint8_t(buf, 0, target);
	_mav_put_uint8_t(buf, 1, type);
	_mav_put_uint8_t(buf, 2, turn);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TURN_LIGHT_LEN);
#else
	mavlink_turn_light_t packet;
	packet.target = target;
	packet.type = type;
	packet.turn = turn;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TURN_LIGHT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TURN_LIGHT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TURN_LIGHT_LEN, MAVLINK_MSG_ID_TURN_LIGHT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TURN_LIGHT_LEN);
#endif
}

/**
 * @brief Encode a turn_light struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param turn_light C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_turn_light_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_turn_light_t* turn_light)
{
	return mavlink_msg_turn_light_pack(system_id, component_id, msg, turn_light->target, turn_light->type, turn_light->turn);
}

/**
 * @brief Send a turn_light message
 * @param chan MAVLink channel to send the message
 *
 * @param target The system ordered to turn on lights
 * @param type Type lights 0: Visible; 1: Infrared
 * @param turn Order turn on lights 1: Turn on; 0: Turn off
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_turn_light_send(mavlink_channel_t chan, uint8_t target, uint8_t type, uint8_t turn)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TURN_LIGHT_LEN];
	_mav_put_uint8_t(buf, 0, target);
	_mav_put_uint8_t(buf, 1, type);
	_mav_put_uint8_t(buf, 2, turn);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TURN_LIGHT, buf, MAVLINK_MSG_ID_TURN_LIGHT_LEN, MAVLINK_MSG_ID_TURN_LIGHT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TURN_LIGHT, buf, MAVLINK_MSG_ID_TURN_LIGHT_LEN);
#endif
#else
	mavlink_turn_light_t packet;
	packet.target = target;
	packet.type = type;
	packet.turn = turn;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TURN_LIGHT, (const char *)&packet, MAVLINK_MSG_ID_TURN_LIGHT_LEN, MAVLINK_MSG_ID_TURN_LIGHT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TURN_LIGHT, (const char *)&packet, MAVLINK_MSG_ID_TURN_LIGHT_LEN);
#endif
#endif
}

#endif

// MESSAGE TURN_LIGHT UNPACKING


/**
 * @brief Get field target from turn_light message
 *
 * @return The system ordered to turn on lights
 */
static inline uint8_t mavlink_msg_turn_light_get_target(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field type from turn_light message
 *
 * @return Type lights 0: Visible; 1: Infrared
 */
static inline uint8_t mavlink_msg_turn_light_get_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field turn from turn_light message
 *
 * @return Order turn on lights 1: Turn on; 0: Turn off
 */
static inline uint8_t mavlink_msg_turn_light_get_turn(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a turn_light message into a struct
 *
 * @param msg The message to decode
 * @param turn_light C-struct to decode the message contents into
 */
static inline void mavlink_msg_turn_light_decode(const mavlink_message_t* msg, mavlink_turn_light_t* turn_light)
{
#if MAVLINK_NEED_BYTE_SWAP
	turn_light->target = mavlink_msg_turn_light_get_target(msg);
	turn_light->type = mavlink_msg_turn_light_get_type(msg);
	turn_light->turn = mavlink_msg_turn_light_get_turn(msg);
#else
	memcpy(turn_light, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_TURN_LIGHT_LEN);
#endif
}
