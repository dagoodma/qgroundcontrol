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
    /** @brief Sets the selective passthrough surfaces for manual control. */
    void setPassthroughSurfaces(bool throttle, bool aileron, bool rudder, bool elevator);
    /** @brief Enables HIL mode (autopilot connects to simulink). */
    void startHil();
    /** @brief Disables  HIL mode. */
    void stopHil();
    bool getHilState() { return (mode & MAV_MODE_FLAG_HIL_ENABLED) > 0; }

public slots:
    /** @brief Receive a MAVLink message from this MAV */
    void receiveMessage(LinkInterface* link, mavlink_message_t message);

signals:
    void midLevelCommandsChanged(int id, double altitude, double airspeed, double turnrate);

};

#endif // _SLUGSMAV_H_

