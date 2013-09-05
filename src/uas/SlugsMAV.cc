/*=====================================================================

QGroundControl Open Source Ground Control Station

(c) 2009, 2010 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>

This file is part of the QGROUNDCONTROL project

    QGROUNDCONTROL is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    QGROUNDCONTROL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with QGROUNDCONTROL. If not, see <http://www.gnu.org/licenses/>.

======================================================================*/

#include "SlugsMAV.h"

#include <QDebug>

SlugsMAV::SlugsMAV(MAVLinkProtocol* mavlink, int id) :
    UAS(mavlink, id)
{
    qDebug() << "Spawning a SLUGS MAV.";
}


/**
* Overrides getNavModeText with SLUGS navigation modes using getSlugsNavModeText().
* @return the mode text of the autopilot
*/
QString SlugsMAV::getNavModeText(int mode) {

#ifdef MAVLINK_ENABLED_SLUGS
    return getSlugsNavModeText(mode);
#else
    return UAS::getNavModeText(mode);
#endif
}

/**
* The mode can be passthrough, landing, liftoff, selective passthrough,
*   lost, returning, mid-level, waypoint, ISR, and line patrol.
* @return the mode text of the autopilot
*/
QString SlugsMAV::getSlugsNavModeText(int mode) {
#ifdef MAVLINK_ENABLED_SLUGS
    switch (mode) {
    case SLUGS_MODE_NONE:
        return QString("NONE");
        break;
    case SLUGS_MODE_PASSTHROUGH:
        return QString("PASSTHROUGH");
        break;
    case SLUGS_MODE_LANDING:
        return QString("LANDING");
        break;
    case SLUGS_MODE_LIFTOFF:
        return QString("LIFTOFF");
        break;
    case SLUGS_MODE_SELECTIVE_PASSTHROUGH:
        return QString("SELECT_PT");
        break;
    case SLUGS_MODE_LOST:
        return QString("LOST");
        break;
    case SLUGS_MODE_RETURNING:
        return QString("RETURNING");
        break;
    case SLUGS_MODE_MID_LEVEL:
        return QString("MID_LEVEL");
        break;
    case SLUGS_MODE_WAYPOINT:
        return QString("WAYPOINT");
        break;
    case SLUGS_MODE_ISR:
        return QString("ISR");
        break;
    case SLUGS_MODE_LINE_PATROL:
        return QString("LINE_PATROL");
        break;
    } // switch
#endif
    return QString("UKNOWN");
}

/**
 * @return Value corresponding to current navigation mode.
 */
int SlugsMAV::getNavMode() {

    return this->navMode;
}

/**
 * @param newNavMode SLUGS navigation mode to set.
 */
void SlugsMAV::setNavMode(int newNavMode) {

    //this->navMode = newNavMode; // don't set here, update on receive hearbeat message from UAS

    mavlink_message_t msg;
    mavlink_msg_set_mode_pack(mavlink->getSystemId(), mavlink->getComponentId(), &msg, (uint8_t)uasId, this->mode, (uint16_t)newNavMode);
    sendMessage(msg);
    qDebug() << "SENDING REQUEST TO SET MODE TO SYSTEM" << uasId << ", REQUEST TO SET SLUGS NAV MODE " << newNavMode;
}

/**
 * This function is called by MAVLink once a complete, uncorrupted (CRC check valid)
 * mavlink packet is received.
 *
 * @param link Hardware link the message came from (e.g. /dev/ttyUSB0 or UDP port).
 *             messages can be sent back to the system via this link
 * @param message MAVLink message, as received from the MAVLink protocol stack
 */
void SlugsMAV::receiveMessage(LinkInterface* link, mavlink_message_t message)
{
    UAS::receiveMessage(link, message);// Let UAS handle the default message set

    if (message.sysid == uasId) {
#ifdef MAVLINK_ENABLED_SLUGS
        switch (message.msgid) {
        // --------------- Custom Slugs Message Handling ---------------
        case MAVLINK_MSG_ID_SENSOR_DIAG:
            // TODO handle sensor diagnostic message
            break;
        case MAVLINK_MSG_ID_CPU_LOAD:
            // TODO handle cpu load message
            break;
        case MAVLINK_MSG_ID_SENSOR_BIAS:
            // TODO handle sensor bias message
            break;
        case MAVLINK_MSG_ID_STATUS_GPS:
            // TODO handle status gps message
            break;
        case MAVLINK_MSG_ID_NOVATEL_DIAG:
            // TODO handle novatel diagnostic message
            break;
        case MAVLINK_MSG_ID_GPS_DATE_TIME:
            // TODO handle gps date and time message
            break;
        case MAVLINK_MSG_ID_DATA_LOG:
            // TODO handle data log message
            break;
        case MAVLINK_MSG_ID_SLUGS_NAVIGATION:
            // TODO handle slugs nav message
            break;
        case MAVLINK_MSG_ID_ISR_LOCATION:
            // TODO handle isr location message
            break;
        case MAVLINK_MSG_ID_PTZ_STATUS:
            // TODO handle pan tilt zoom message
            break;
        case MAVLINK_MSG_ID_VOLT_SENSOR:
            // TODO handle isr location message
            break;
        case MAVLINK_MSG_ID_CTRL_SRFC_PT:
            // TODO handle control surface passthrough message
            break;
        case MAVLINK_MSG_ID_BOOT:
            // TODO handle boot message
            break;
        case MAVLINK_MSG_ID_MID_LVL_CMDS:
            qDebug() << "Got mid-level command message: altitude=" << mavlink_msg_mid_lvl_cmds_get_hCommand(&message)
                     << ", airspeed=" << mavlink_msg_mid_lvl_cmds_get_uCommand(&message)
                     << ", turnrate=" << mavlink_msg_mid_lvl_cmds_get_rCommand(&message);
            emit midLevelCommandsChanged(uasId, (double)mavlink_msg_mid_lvl_cmds_get_hCommand(&message),
                                         (double)mavlink_msg_mid_lvl_cmds_get_uCommand(&message),
                                         (double)mavlink_msg_mid_lvl_cmds_get_rCommand(&message));
            break;
        } // switch
#endif
    } // (message.sysid == uasId)
}

void SlugsMAV::requestMidLevelCommands() {
#ifdef MAVLINK_ENABLED_SLUGS
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(mavlink->getSystemId(), mavlink->getComponentId(), &msg, (uint8_t)uasId, 0,
                                  MAV_CMD_GET_MID_LEVEL_COMMANDS,0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
    sendMessage(msg);
    qDebug() << "Requesting mid-level commands from system " << uasId;
#endif
}

/**
 * @param altitude To hold in meters.
 * @param airspeed To hold in meters per second.
 * @param turnrate To hold in radians per second.
 */
void SlugsMAV::setMidLevelCommands(double altitude, double airspeed, double turnrate) {
#ifdef MAVLINK_ENABLED_SLUGS
    mavlink_message_t msg;
    mavlink_msg_mid_lvl_cmds_pack(mavlink->getSystemId(), mavlink->getComponentId(), &msg, (uint8_t)uasId,
                                  static_cast<float>(altitude), static_cast<float>(airspeed), static_cast<float>(turnrate));
    sendMessage(msg);

    qDebug() << "Setting mid-level commands for system " << uasId << ": altitude=" << altitude
             << ", airspeed=" << airspeed << ", turnrate=" << turnrate;
#endif
}

/**
 * @param throttle Whether pilot console controls throttle.
 * @param ailerons Whether pilot console controls ailerons.
 * @param rudder Whether pilot console controls rudder.
 * @param elevator Whether pilot console controls elevator.
 */
void SlugsMAV::setPassthroughSurfaces(bool throttle, bool ailerons, bool rudder, bool elevator) {
#ifdef MAVLINK_ENABLED_SLUGS
    uint16_t passthroughBitfield = 0;
    if (throttle) passthroughBitfield += CONTROL_SURFACE_FLAG_THROTTLE;
    if (ailerons) passthroughBitfield += CONTROL_SURFACE_FLAG_LEFT_AILERON + CONTROL_SURFACE_FLAG_RIGHT_AILERON;
    if (rudder) passthroughBitfield += CONTROL_SURFACE_FLAG_RUDDER;
    if (elevator) passthroughBitfield += CONTROL_SURFACE_FLAG_LEFT_ELEVATOR + CONTROL_SURFACE_FLAG_RIGHT_ELEVATOR;

    qDebug() << "Changing passthrough surface selection " << passthroughBitfield;

    mavlink_message_t msg;
    mavlink_msg_ctrl_srfc_pt_pack(mavlink->getSystemId(), mavlink->getComponentId(), &msg, 0, passthroughBitfield);
    sendMessage(msg);
#endif
}

void SlugsMAV::startHil()
{
    if (getHilState()) return;
    //hilEnabled = true; // this will be set on next heartbeat
    mavlink_message_t msg;
    mavlink_msg_set_mode_pack(mavlink->getSystemId(), mavlink->getComponentId(), &msg, this->getUASID(), mode | MAV_MODE_FLAG_HIL_ENABLED, navMode);
    sendMessage(msg);
}

void SlugsMAV::stopHil()
{
    if (!getHilState()) return;
    mavlink_message_t msg;
    mavlink_msg_set_mode_pack(mavlink->getSystemId(), mavlink->getComponentId(), &msg, this->getUASID(), mode & ~MAV_MODE_FLAG_HIL_ENABLED, navMode);
    sendMessage(msg);
    //hilEnabled = false; // this will be set on next heartbeat
}


void SlugsMAV::setIsrLocation(double lat, double lon, double alt) {
    mavlink_message_t msg;
    mavlink_msg_isr_location_pack(mavlink->getSystemId(), mavlink->getComponentId(), &msg, this->getUASID(),
                                  (float)lat, (float)lon, (float)alt,0,0,0);
    sendMessage(msg);
    qDebug() << "Sent ISR location to mav: lat=" << lat << ", lon=" << lon << ", alt=" << alt;
}
