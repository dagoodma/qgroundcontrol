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
    void setSystemType(UASInterface* uas, unsigned int systemType);

public slots:
    void setUAS(UASInterface* uas);
    void receiveHeartbeat(UASInterface* uas);
    void heartbeatTimeout(bool timeout, unsigned int ms);
//    void updateName(const QString& name);
//    void updateLocalPosition(UASInterface* uas, double x, double y, double z, quint64 usec);
//    void updateGlobalPosition(UASInterface* uas, double lon, double lat, double alt, quint64 usec);
//    void updateGpsFix(UASInterface* uas, int fixQuality);
//    void updateSpeed(UASInterface*, double x, double y, double z, quint64 usec);
//    //void updateBattery(UASInterface* uas, double voltage, double current, double percent, int seconds);
//    //void updateLoad(UASInterface* uas, double ctrlLoad, double sensLoad);
//    //void setBatterySpecs();
    void refresh();
    /** @brief Start widget updating */
    //void showEvent(QShowEvent* event);
    /** @brief Stop widget updating */
    //void hideEvent(QHideEvent* event);
//    void contextMenuEvent(QContextMenuEvent* event);

protected:
//    void changeEvent(QEvent *e);
    UASInterface* uas;
    QTimer* refreshTimer;
    QColor heartbeatColor;
    quint64 startTime;
    int fixQuality;
    bool timeout;
    bool iconIsRed;
    bool disconnected;
    float chargeLevel;
    float ctrlLoad;
    float sensLoad;
    QAction* setBatterySpecsAction;
    float z;
    float totalSpeed;
    float alt;
    bool localFrame;
    bool globalFrameKnown;
    static const int updateInterval = 800;
    static const int errorUpdateInterval = 200;
    bool lowPowerModeEnabled; ///< Low power mode reduces update rates
    unsigned int generalUpdateCount; ///< Skip counter for updates

private:
    Ui::slugsStatus *m_ui;
    virtual void paintEvent(QPaintEvent *);

};

#endif // SLUGSSTATUSWIDGET_H
