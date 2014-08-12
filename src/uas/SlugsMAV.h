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

#ifndef _SLUGSMAV_H_
#define _SLUGSMAV_H_

#include "UAS.h"
#include "mavlink.h"
#include <QTimer>
#include <QObject>

#ifdef MAVLINK_ENABLED_SLUGS
#include "slugs/slugs.h"
#endif

class SlugsMAV : public UAS
{
    Q_OBJECT
    Q_INTERFACES(UASInterface)

public:
    SlugsMAV(MAVLinkProtocol* mavlink, int id = 0);
    //~SlugsMAV(void);
    /** @brief Gets current navigation mode value. */
    int getNavMode();
    /** @brief Sets the SLUGS navigation mode. */
    void setNavMode(int navNavMode);
    /** @brief Returns the name of the SLUGS navigation mode. */
    QString getNavModeText(int mode);
    /** @brief Helper for getNavModeTex. */
    static QString getSlugsNavModeText(int mode);
    /** @brief Sends a request for current mid-level commands. */
    void requestMidLevelCommands();
    /** @brief Sets mid-level commands. */
    void setMidLevelCommands(double altitude, double airspeed, double turnrate);
    /** @brief Reads mid-level commands from EEPROM. */
    void readMidLevelCommandsFromEeprom();
    /** @brief Writes mid-level commands to EEPROM. */
    void writeMidLevelCommandsToEeprom();
    /** @brief Sets the selective passthrough surfaces for manual control. */
    void setPassthroughSurfaces(bool throttle, bool aileron, bool rudder, bool elevator);
    /** @brief Enables HIL mode (autopilot connects to simulink). */
    void startHil();
    /** @brief Disables  HIL mode. */
    void stopHil();
    /** @brief Returns whether the HIL flag is set on mode. */
    bool getHilState() { return (mode & MAV_MODE_FLAG_HIL_ENABLED) > 0; }
    /** @brief Sets the current ISR location. */
    void setIsrLocation(double lat, double lon, double alt);
    /** @brief Whether the MAV is returning to base or not. */
    bool IsReturning() {
        return isReturning;
    }

public slots:
    /** @brief Receive a MAVLink message from this MAV */
    void receiveMessage(LinkInterface* link, mavlink_message_t message);
    /** @brief Send a MAVLink ping request message */
    void sendPing(void);

signals:
    void midLevelCommandsChanged(int id, double altitude, double airspeed, double turnrate);
    void loadsChanged(int id, double ctrlLoad, double sensLoad);
    void airSpeedChanged(int id, float airSpeed);
    void gpsFixChanged(int id, unsigned int gpsQuality);

private:
    unsigned int gpsFixQuality;
    bool isReturning;
    bool sendPingRequests;
    QTimer* pingTimer;
    int pingRate;

};

#endif // _SLUGSMAV_H_

