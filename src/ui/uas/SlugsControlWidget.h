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

/**
 * @file
 *   @brief Definition of class SlugsControlWidget
 *
 *   @author David Goodman <dagoodman@soe.ucsc.edu>
 *
 */

#ifndef _SLUGSCONTROLWIDGET_H_
#define _SLUGSCONTROLWIDGET_H_

#include <QWidget>
#include "MainWindow.h"
#include <QMap>
#include <QString>
#include <QPushButton>
#include <ui_SlugsControl.h>
#include <UASInterface.h>
#include <SlugsMAV.h>
#include <UASManager.h>
#include <UAS.h>
//#include "QGC.h"

/**
 * @brief Widget for controlling a SLUGS MAV.
 */
class SlugsControlWidget : public QWidget
{
    Q_OBJECT

public:
    SlugsControlWidget(QWidget *parent = 0);
    ~SlugsControlWidget();
    /** @brief Transmits the navigation mode to the MAV. */
    void transmitNavigationMode(int mode);

protected slots:
    /** @brief Set the system this widget controls. */
    void setUAS(UASInterface* uas);
    /** @brief Arms or disarms the MAV's engines. */
    void armDisarmButtonClicked();
    /** @brief Updates armed/disarmed state of MAV. */
    void updateArmDisarm(bool state);
    /** @brief Requests mid-level commands from the MAV. */
    void getMidLevelButtonClicked();
    /** @brief Sends specifed mid-level commands to the MAV. */
    void setMidLevelButtonClicked();
    /** @brief Request to write mid-level commands from volatile to non-volatile memory. */
    void writeMidLevelButtonClicked();
    /** @brief  Request to read mid-level commands from non-volatile memory and load into volatile memory. */
    void readMidLevelButtonClicked();
    /** @brief Sets the mid-level parameters received from the MAV. */
    void updateMidLevelParameters(int uas, double altitude, double airspeed, double turnrate);
    /** @brief Sends selected control surfaces to MAV for selective passthrough mode. */
    void setPassthroughButtonClicked();
    /** @brief Sets the navigation mode and trasmits it to the MAV. */
    void navigationModeButtonClicked(int mode);
    /** @brief Sets the navigation mode and styles the corresponding button. */
    void updateNavMode(int uas, int mode,QString description);

#ifdef MAVLINK_ENABLED_SLUGS
    void waypointModeButtonClicked() { navigationModeButtonClicked(SLUGS_MODE_WAYPOINT); }
    void isrModeButtonClicked()  {

        if (!uas) return;
        UASInterface* mav = UASManager::instance()->getUASForId(this->uas);
        if (!mav) return;
        SlugsMAV* slugsMav = static_cast<SlugsMAV*>(mav);

        // Loiter at current location
        slugsMav->setIsrLocation(slugsMav->getLatitude(),slugsMav->getLongitude(),slugsMav->getAltitudeAMSL());
        navigationModeButtonClicked(SLUGS_MODE_ISR);
    }
    void selectivePassthroughModeButtonClicked()  { navigationModeButtonClicked(SLUGS_MODE_SELECTIVE_PASSTHROUGH); }
    void passthroughModeButtonClicked()  { navigationModeButtonClicked(SLUGS_MODE_PASSTHROUGH); }
    void midLevelModeButtonClicked() { navigationModeButtonClicked(SLUGS_MODE_MID_LEVEL); }
    void linePatrolModeButtonClicked() { navigationModeButtonClicked(SLUGS_MODE_LINE_PATROL); }
#endif

protected:
    int uas;                    ///< Reference to the current uas
    int uasNavigationMode; ///< Current navigation mode
    int uasNavigationModeSent;   ///< Current navigation mode sent
    bool uasMidLevelRequestSent;    ///< Request for mid-level commands sent
    //bool engineOn;              ///< Engine state

private:
    Ui::slugsControl ui;
    //MainWindow* mainWindow;
    QMap<int, QPushButton*> navModeButtons;


};

#endif // _SLUGSCONTROLWIDGET_H_
