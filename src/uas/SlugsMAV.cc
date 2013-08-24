#include "SlugsMAV.h"

#include <QDebug>

SlugsMAV::SlugsMAV(MAVLinkProtocol* mavlink, int id) :
    UAS(mavlink, id)
{
    qDebug() << "Spawning a SLUGS MAV.";
    /*
    widgetTimer = new QTimer (this);
    widgetTimer->setInterval(SLUGS_UPDATE_RATE);

    connect (widgetTimer, SIGNAL(timeout()), this, SLOT(emitSignals()));
    widgetTimer->start();
*/
    /*
    memset(&mlRawImuData ,0, sizeof(mavlink_raw_imu_t));// clear all the state structures

#ifdef MAVLINK_ENABLED_SLUGS


    memset(&mlGpsData, 0, sizeof(mavlink_gps_raw_t));
    memset(&mlCpuLoadData, 0, sizeof(mavlink_cpu_load_t));
    memset(&mlAirData, 0, sizeof(mavlink_air_data_t));
    memset(&mlSensorBiasData, 0, sizeof(mavlink_sensor_bias_t));
    memset(&mlDiagnosticData, 0, sizeof(mavlink_diagnostic_t));
    memset(&mlBoot ,0, sizeof(mavlink_boot_t));
    memset(&mlGpsDateTime ,0, sizeof(mavlink_gps_date_time_t));
    memset(&mlApMode ,0, sizeof(mavlink_set_mode_t));
    memset(&mlNavigation ,0, sizeof(mavlink_slugs_navigation_t));
    memset(&mlDataLog ,0, sizeof(mavlink_data_log_t));
    memset(&mlPassthrough ,0, sizeof(mavlink_ctrl_srfc_pt_t));
    memset(&mlActionAck,0, sizeof(mavlink_action_ack_t));
    memset(&mlAction ,0, sizeof(mavlink_slugs_action_t));

    memset(&mlScaled ,0, sizeof(mavlink_scaled_imu_t));
    memset(&mlServo ,0, sizeof(mavlink_servo_output_raw_t));
    memset(&mlChannels ,0, sizeof(mavlink_rc_channels_raw_t));

    updateRoundRobin = 0;
    uasId = id;
#endif
*/
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
    /*case SLUGS_MODE_SELECTIVE_PASSTHROUGH:
        return QString("SELECT_PT");
        break;
    */
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
        /* Handled in UAS.cc where
         *  navModeChanged(uasId, state.custom_mode, getNavModeText(state.custom_mode))
         * is emitted.
        // ----------- Handle SLUGS Specific Common Messages -----------
        case MAVLINK_MSG_ID_HEARTBEAT:
            // Check custom mode for slugs nav mode
            SLUGS_MODE navModeTemp = mavlink_msg_heartbeat_get_custom_mode(message);
            if (navMode != navModeTemp) {
                navMode = navModeTemp;
                emit slugsNavModeChanged(uasId, navModeTemp, getNavModeText(navModeTemp));
            }
            break;
         */

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
            // TODO handle mid-level command message
            break;
        }


        /*

        case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
            mavlink_msg_rc_channels_raw_decode(&message, &mlChannels);
            break;

            switch (mlAction.actionId) {
            case SLUGS_ACTION_EEPROM:
                if (mlAction.actionVal == SLUGS_ACTION_FAIL) {
                    emit textMessageReceived(message.sysid, message.compid, 255, "EEPROM Write Fail, Data was not saved in Memory!");
                }
                break;

            case SLUGS_ACTION_PT_CHANGE:
                if (mlAction.actionVal == SLUGS_ACTION_SUCCESS) {
                    emit textMessageReceived(message.sysid, message.compid, 0, "Passthrough Succesfully Changed");
                }
                break;

            case SLUGS_ACTION_MLC_CHANGE:
                if (mlAction.actionVal == SLUGS_ACTION_SUCCESS) {
                    emit textMessageReceived(message.sysid, message.compid, 0, "Mid-level Commands Succesfully Changed");
                }
                break;
            }

            //break;

        default:
            //        qDebug() << "\nSLUGS RECEIVED MESSAGE WITH ID" << message.msgid;
            break;
        }
        */
#endif
    } // (message.sysid == uasId)
}


/*
void SlugsMAV::emitSignals (void)
{
#ifdef MAVLINK_ENABLED_SLUGS
    switch(updateRoundRobin) {
    case 1:
        emit slugsCPULoad(uasId, mlCpuLoadData);
        emit slugsSensorBias(uasId,mlSensorBiasData);
        break;

    case 2:
        emit slugsAirData(uasId, mlAirData);
        emit slugsDiagnostic(uasId,mlDiagnosticData);

        break;

    case 3:
        emit slugsNavegation(uasId, mlNavigation);
        emit slugsDataLog(uasId, mlDataLog);
        break;

    case 4:
        emit slugsGPSDateTime(uasId, mlGpsDateTime);

        break;

    case 5:
        emit slugsActionAck(uasId,mlActionAck);
        emit emitGpsSignals();
        break;

    case 6:
        emit slugsChannels(uasId, mlChannels);
        emit slugsServo(uasId, mlServo);
        emit slugsScaled(uasId, mlScaled);

        break;
    }

    emit slugsAttitude(uasId, mlAttitude);
    emit attitudeChanged(this,
                         mlAttitude.roll,
                         mlAttitude.pitch,
                         mlAttitude.yaw,
                         0.0);
#endif

    emit slugsRawImu(uasId, mlRawImuData);


    // wrap around
    updateRoundRobin = updateRoundRobin > 10? 1: updateRoundRobin + 1;


}
*/

/*

#ifdef MAVLINK_ENABLED_SLUGS
void SlugsMAV::emitGpsSignals (void)
{

    // qDebug()<<"After Emit GPS Signal"<<mlGpsData.fix_type;


    //ToDo Uncomment if. it was comment only to test

// if (mlGpsData.fix_type > 0){
    emit globalPositionChanged(this,
                               mlGpsData.lon,
                               mlGpsData.lat,
                               mlGpsData.alt,
                               0.0);

    emit slugsGPSCogSog(uasId,mlGpsData.hdg, mlGpsData.v);

}



#endif // MAVLINK_ENABLED_SLUGS
*/
