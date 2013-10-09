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

#ifndef SLUGSSTATUSWIDGET_H
#define SLUGSSTATUSWIDGET_H

#include <QWidget>
#include <QString>
#include <QTimer>
#include <QMouseEvent>
#include <UASInterface.h>
#include "ui_SlugsStatus.h"


class SlugsStatusWidget : public QWidget
{
    Q_OBJECT
public:
    SlugsStatusWidget(QWidget *parent = 0);
    ~SlugsStatusWidget();

public slots:
    void setUAS(UASInterface* uas);
    void receiveHeartbeat(UASInterface* uas);
    void heartbeatTimeout(bool timeout, unsigned int ms);
    void setBatterySpecs();
    void updateName(const QString& name);
    void updateLocalPosition(UASInterface* uas, double x, double y, double z, quint64 usec);
    void updateGlobalPosition(UASInterface* uas, double lon, double lat, double alt, quint64 usec);
    void updateBattery(UASInterface* uas, double voltage, double current, double percent, int seconds);
    void updateGpsFix(int id, unsigned int fixQuality);
    void updateSpeed(int id, float airSpeed);
    void updateLoad(int id, double ctrlLoad, double sensLoad);
    void refresh();
    /** @brief Start widget updating */
    void showEvent(QShowEvent* event);
    /** @brief Stop widget updating */
    void hideEvent(QHideEvent* event);
    void contextMenuEvent(QContextMenuEvent* event);

protected:
    void changeEvent(QEvent *e);
    UASInterface* uas;
    QTimer* refreshTimer;
    QColor heartbeatColor;
    QColor gpsColor;
    quint64 startTime;
    int fixQuality;
    bool timeout;
    bool iconIsRed;
    bool disconnected;
    bool altitudeIsRed;
    bool speedIsRed;
    bool altitudeIsWarning;
    bool speedIsWarning;
    float batteryCharge;
    float batteryVoltage;
    float ctrlLoad;
    float sensLoad;
    QAction* setBatterySpecsAction;
    float z;
    float airSpeed;
    float alt;
    bool localFrame;
    bool globalFrameKnown;
    static const int updateDisplayIntervalMultiplier = 4;
    static const int updateInterval = 200;
    static const int updateIntervalLowPower = 200;
    static const int errorUpdateInterval = 200;
    // These are set in constructor
    float airSpeedLowLimit; ///< Lower limit for airspeed warning(red)
    float airSpeedHighLimit; ///< Upper limit for airspeed warning (red)
    float airSpeedLimitThreshold; ///< Threshold from limits for warning (yellow)
    float altitudeLowLimit; ///< Lower limit for altitude warning (red)
    float altitudeLimitThreshold; ///< Threshold from limits for warning (yellow)
    float altitudeHighLimit; ///< Upper limit for altitude warning (red)
    bool lowPowerModeEnabled; ///< Low power mode reduces update rates
    unsigned int generalUpdateCount; ///< Skip counter for updates

private:
    Ui::slugsStatus *m_ui;
    virtual void paintEvent(QPaintEvent *);

};

#endif // SLUGSSTATUSWIDGET_H
