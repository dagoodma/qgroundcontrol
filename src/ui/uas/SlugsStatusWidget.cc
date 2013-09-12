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
 *   @brief Definition of widget for viewing status of a SLUGS MAV
 *
 *   @author David Goodman <dagoodman@soe.ucsc.edu>
 *
 */

#include <cmath>
#include <QDateTime>
#include <QDebug>
#include <QMenu>
#include <QInputDialog>

#include "QGC.h"
#include "UASManager.h"
#include "MainWindow.h"
#include "SlugsStatusWidget.h"
#include "SlugsMAV.h"



SlugsStatusWidget::SlugsStatusWidget(QWidget *parent) :
    QWidget(parent),
    uas(0),
    startTime(0),
    timeout(false),
    disconnected(true),
    totalSpeed(0),
    alt(0),
    z(0),
    localFrame(false),
    globalFrameKnown(false),
    lowPowerModeEnabled(true),
    generalUpdateCount(0),
    fixQuality(0),
    chargeLevel(0),
    ctrlLoad(0),
    sensLoad(0),
    setBatterySpecsAction(new QAction(tr("Set Battery Options"), this)),
    m_ui(new Ui::slugsStatus)
{

    m_ui->setupUi(this);

    refreshTimer = new QTimer(this);
    connect(refreshTimer, SIGNAL(timeout()), this, SLOT(refresh()));
    connect(UASManager::instance(), SIGNAL(activeUASSet(UASInterface*)), this, SLOT(setUAS(UASInterface*)));
    if (UASManager::instance()->getActiveUAS())
    {
        setUAS(UASManager::instance()->getActiveUAS());
    }
}

SlugsStatusWidget::~SlugsStatusWidget() {
    //if (refreshTimer) disconnect(refreshTimer, SIGNAL(timeout()), this, SLOT(refresh()));
    //disconnect(refreshTimer, SIGNAL(timeout()), this, SLOT(refresh()));
    //setUAS(NULL);


    //delete m_ui;
}


void SlugsStatusWidget::setUAS(UASInterface* uas) {

    if (this->uas) {

        // Setup communication
        // Heartbeat fade
        disconnect(this->uas, SIGNAL(heartbeatTimeout(bool, unsigned int)), this, SLOT(heartbeatTimeout(bool, unsigned int)));
        disconnect(this->uas, SIGNAL(heartbeat(UASInterface*)), this, SLOT(receiveHeartbeat(UASInterface*)));

        // Velocity and altitude
//        //disconnect(this->uas, SIGNAL(localPositionChanged(UASInterface*,double,double,double,quint64)), this, SLOT(updateLocalPosition(UASInterface*,double,double,double,quint64)));
//        //disconnect(this->uas, SIGNAL(globalPositionChanged(UASInterface*,double,double,double,quint64)), this, SLOT(updateGlobalPosition(UASInterface*,double,double,double,quint64)));
//        disconnect(this->uas, SIGNAL(velocityChanged_NED(UASInterface*,double,double,double,quint64)), this, SLOT(updateSpeed(UASInterface*,double,double,double,quint64)));

        // System type and battery related
        disconnect(this->uas, SIGNAL(systemTypeSet(UASInterface*,uint)), this, SLOT(setSystemType(UASInterface*,uint)));
        //disconnect(this->uas, SIGNAL(batteryChanged(UASInterface*, double, double, double, int)), this, SLOT(updateBattery(UASInterface*, double, double, double, int)));
        disconnect(setBatterySpecsAction, SIGNAL(triggered()), this, SLOT(setBatterySpecs()));

        // Name changes
//        disconnect(uas, SIGNAL(nameChanged(QString)), this, SLOT(updateName(QString)));

        // Slugs only
        if(this->uas->getAutopilotType() == MAV_AUTOPILOT_SLUGS) {
            disconnect((SlugsMAV*)(this->uas), SIGNAL(gpsFixChanged(UASInterface*, int)), this, SLOT(updateGpsFix(UASInterface*,int)));
            disconnect(this->uas, SIGNAL(loadChanged(UASInterface*, double)), this, SLOT(updateLoad(UASInterface*, double)));
        }

        // Disconnect and stop timer
        if (!uas) {
            disconnected = true;
            refresh();
            refreshTimer->stop();
        }

    }
    if (uas) {
        // Setup communication

        // Velocity and altitude
//        //disconnect(this->uas, SIGNAL(localPositionChanged(UASInterface*,double,double,double,quint64)), this, SLOT(updateLocalPosition(UASInterface*,double,double,double,quint64)));
//        //disconnect(this->uas, SIGNAL(globalPositionChanged(UASInterface*,double,double,double,quint64)), this, SLOT(updateGlobalPosition(UASInterface*,double,double,double,quint64)));
//        disconnect(this->uas, SIGNAL(velocityChanged_NED(UASInterface*,double,double,double,quint64)), this, SLOT(updateSpeed(UASInterface*,double,double,double,quint64)));

        // System type and battery related
        connect(uas, SIGNAL(systemTypeSet(UASInterface*,uint)), this, SLOT(setSystemType(UASInterface*,uint)));
        //disconnect(this->uas, SIGNAL(batteryChanged(UASInterface*, double, double, double, int)), this, SLOT(updateBattery(UASInterface*, double, double, double, int)));
        connect(setBatterySpecsAction, SIGNAL(triggered()), this, SLOT(setBatterySpecs()));

        // Name changes
//        disconnect(uas, SIGNAL(nameChanged(QString)), this, SLOT(updateName(QString)));

        // Slugs only
        if(uas->getAutopilotType() == MAV_AUTOPILOT_SLUGS) {
            connect((SlugsMAV*)(uas), SIGNAL(gpsFixChanged(UASInterface*, int)), this, SLOT(updateGpsFix(UASInterface*,int)));
            connect(uas, SIGNAL(loadChanged(UASInterface*, double)), this, SLOT(updateLoad(UASInterface*, double)));
        }

        // Heartbeat fade
        connect(uas, SIGNAL(heartbeat(UASInterface*)), this, SLOT(receiveHeartbeat(UASInterface*)));
        connect(uas, SIGNAL(heartbeatTimeout(bool, unsigned int)), this, SLOT(heartbeatTimeout(bool, unsigned int)));
        if (lowPowerModeEnabled)
        {
            refreshTimer->start(updateInterval*3);
        }
        else
        {
            refreshTimer->start(updateInterval);
        }

        // Style heartbeat and gps indicators
        heartbeatColor = Qt::green;
        QString colorstyle("QLabel { background-color: %1; }");
        m_ui->typeLabel->setStyleSheet(colorstyle.arg(heartbeatColor.name()));

        setSystemType(uas, uas->getSystemType());
    } // uas

    this->uas = uas;

}


void SlugsStatusWidget::heartbeatTimeout(bool timeout, unsigned int ms)
{
    Q_UNUSED(ms);
    this->timeout = timeout;
}


void SlugsStatusWidget::receiveHeartbeat(UASInterface* uas)
{
    Q_UNUSED(uas);
    //heartbeatColor = uas->getColor();
    heartbeatColor = Qt::green;
    QString colorstyle("QLabel { background-color: %1; }");
    m_ui->heartBeatLabel->setStyleSheet(colorstyle.arg(heartbeatColor.name()));

    // If we're returning from a disconnection, recolor things properly.
    if (disconnected)
    {
        //updateActiveUAS(this->uas, this->isActive);
        disconnected = false;
    }
    timeout = false;
}

/*
void SlugsStatusWidget::updateName(const QString& name)
{
    //if (uas) m_ui->nameLabel->setText(name);
}

void SlugsStatusWidget::updateGpsFix(UASInterface* uas, int fixQuality) {
    if (!uas) return;

    this->fixQuality = fixQuality;
    if (fixQuality > 0) {
        // TODO make GPS indicator green

    }
    else {

    }

}
*/

/**
 * The current system type is represented through the system icon.
 *
 * @param uas Source system, has to be the same as this->uas
 * @param systemType type ID, following the MAVLink system type conventions
 * @see http://pixhawk.ethz.ch/software/mavlink
 */
void SlugsStatusWidget::setSystemType(UASInterface* uas, unsigned int systemType)
{
    if (uas && systemType && (uas == this->uas))
    {
        // Set matching icon
        switch (systemType)
        {
        case MAV_TYPE_GENERIC:
            m_ui->typeLabel->setPixmap(QPixmap(":/files/images/mavs/generic.svg"));
            break;
        case MAV_TYPE_FIXED_WING:
            m_ui->typeLabel->setPixmap(QPixmap(":/files/images/mavs/fixed-wing.svg"));
            break;
        case MAV_TYPE_QUADROTOR:
            m_ui->typeLabel->setPixmap(QPixmap(":/files/images/mavs/quadrotor.svg"));
            break;
        case MAV_TYPE_COAXIAL:
            m_ui->typeLabel->setPixmap(QPixmap(":/files/images/mavs/coaxial.svg"));
            break;
        case MAV_TYPE_HELICOPTER:
            m_ui->typeLabel->setPixmap(QPixmap(":/files/images/mavs/helicopter.svg"));
            break;
        case MAV_TYPE_ANTENNA_TRACKER:
            m_ui->typeLabel->setPixmap(QPixmap(":/files/images/mavs/unknown.svg"));
            break;
        case MAV_TYPE_GCS: {
                // A groundstation is a special system type, update widget
                QString result;
                //m_ui->nameLabel->setText(tr("GCS ") + result.sprintf("%03d", uas->getUASID()));
                m_ui->typeLabel->setPixmap(QPixmap(":/files/images/mavs/groundstation.svg"));
            }
            break;
        case MAV_TYPE_AIRSHIP:
            m_ui->typeLabel->setPixmap(QPixmap(":files/images/mavs/airship.svg"));
            break;
        case MAV_TYPE_FREE_BALLOON:
            m_ui->typeLabel->setPixmap(QPixmap(":files/images/mavs/free-balloon.svg"));
            break;
        case MAV_TYPE_ROCKET:
            m_ui->typeLabel->setPixmap(QPixmap(":files/images/mavs/rocket.svg"));
            break;
        case MAV_TYPE_GROUND_ROVER:
            m_ui->typeLabel->setPixmap(QPixmap(":files/images/mavs/ground-rover.svg"));
            break;
        case MAV_TYPE_SURFACE_BOAT:
            m_ui->typeLabel->setPixmap(QPixmap(":files/images/mavs/surface-boat.svg"));
            break;
        case MAV_TYPE_SUBMARINE:
            m_ui->typeLabel->setPixmap(QPixmap(":files/images/mavs/submarine.svg"));
            break;
        case MAV_TYPE_HEXAROTOR:
            m_ui->typeLabel->setPixmap(QPixmap(":files/images/mavs/hexarotor.svg"));
            break;
        case MAV_TYPE_OCTOROTOR:
            m_ui->typeLabel->setPixmap(QPixmap(":files/images/mavs/octorotor.svg"));
            break;
        case MAV_TYPE_TRICOPTER:
            m_ui->typeLabel->setPixmap(QPixmap(":files/images/mavs/tricopter.svg"));
            break;
        case MAV_TYPE_FLAPPING_WING:
            m_ui->typeLabel->setPixmap(QPixmap(":files/images/mavs/flapping-wing.svg"));
            break;
        case MAV_TYPE_KITE:
            m_ui->typeLabel->setPixmap(QPixmap(":files/images/mavs/kite.svg"));
            break;
        default:
            m_ui->typeLabel->setPixmap(QPixmap(":/files/images/mavs/unknown.svg"));
            break;
        }
    }
}

/*
void SlugsStatusWidget::updateLocalPosition(UASInterface* uas, double x, double y, double z, quint64 usec)
{
    Q_UNUSED(usec);
    Q_UNUSED(uas);
    Q_UNUSED(x);
    Q_UNUSED(y);
    this->z = z;
    localFrame = true;
}

void SlugsStatusWidget::updateGlobalPosition(UASInterface* uas, double lon, double lat, double alt, quint64 usec)
{
    Q_UNUSED(uas);
    Q_UNUSED(usec);
    Q_UNUSED(lat);
    Q_UNUSED(lon);
    this->alt = alt;
    globalFrameKnown = true;
}

void SlugsStatusWidget::updateSpeed(UASInterface*, double x, double y, double z, quint64 usec)
{
    Q_UNUSED(usec);
    totalSpeed = sqrt(x*x + y*y + z*z);
}
*/

/*
void SlugsStatusWidget::updateBattery(UASInterface* uas, double voltage, double current, double percent, int seconds)
{
    Q_UNUSED(voltage);
    Q_UNUSED(current);
    if (this->uas == uas)
    {
        //timeRemaining = seconds;
        chargeLevel = percent;
    }
}
*/

/*
void SlugsStatusWidget::updateLoad(UASInterface* uas, double ctrlLoad, double sensLoad)
{
    if (this->uas == uas)
    {
        this->ctrlLoad = ctrlLoad;
        this->sensLoad = sensLoad;
    }
}
*/
/*
void SlugsStatusWidget::contextMenuEvent (QContextMenuEvent* event)
{
    QMenu menu(this);
    //menu.addAction(setBatterySpecsAction);
    menu.exec(event->globalPos());
}
*/

/*
void SlugsStatusWidget::setBatterySpecs()
{
    if (uas)
    {
        bool ok;
        QString newName = QInputDialog::getText(this, tr("Set Battery Specifications for %1").arg(uas->getUASName()),
                                                tr("Specs: (empty,warn,full), e.g. (9V,9.5V,12.6V) or just warn level in percent (e.g. 15%) to use estimate from MAV"), QLineEdit::Normal,
                                                uas->getBatterySpecs(), &ok);

        if (ok && !newName.isEmpty()) uas->setBatterySpecs(newName);
    }
}

*/

void SlugsStatusWidget::refresh()
{
    if (!disconnected && generalUpdateCount == 4 && uas)
    {
        // Update interface at 1/4th of speed
        generalUpdateCount = 0;

        // State
        //m_ui->stateLabel->setText(state);
        //m_ui->statusTextLabel->setText(stateDesc);

        // Battery
        //m_ui->batteryBar->setValue(static_cast<int>(this->chargeLevel));
        //m_ui->ctrlLoadBar->setValue(static_cast<int>(this->ctrlLoad));
        //m_ui->sensLoadBar->setValue(static_cast<int>(this->sensLoad));

        // Altitude and speed
        //m_ui->altitudeLabel->setText(QString("%1 m").arg(alt, 6, 'f', 1, '0'));
        //QString speed("%1 m/s");
        //m_ui->speedLabel->setText(speed.arg(totalSpeed, 4, 'f', 1, '0'));

    }
    generalUpdateCount++;

    if (timeout)
    {
        // CRITICAL CONDITION, NO HEARTBEAT
        disconnected = true;

        QColor warnColor = Qt::red;

        refreshTimer->setInterval(errorUpdateInterval);
        refreshTimer->start();

        QString style = QString("QLabel {background-color: %1;}").arg(warnColor.name());
        m_ui->heartBeatLabel->setStyleSheet(style);
    }
    else
    {
        // If we're not in low power mode, add the additional visual effect of
        // fading out the color of the heartbeat for this UAS.

        //if (!lowPowerModeEnabled)
        //{
        heartbeatColor = heartbeatColor.darker(110);
        QString colorstyle("QLabel {background-color: %1;}");
        m_ui->heartBeatLabel->setStyleSheet(colorstyle.arg(heartbeatColor.name()));
        refreshTimer->setInterval(updateInterval);
        refreshTimer->start();
        //}
    }


    QColor typeColor = (disconnected)? Qt::red : Qt::green;
    QString typeStyle = QString("QLabel {background-color: %1;}").arg(typeColor.name());
    m_ui->typeLabel->setStyleSheet(typeStyle);
} // refresh()



/**
 * Implement paintEvent() so that stylesheets work for our custom widget.
 */
void SlugsStatusWidget::paintEvent(QPaintEvent *)
{
    QStyleOption opt;
    opt.init(this);
    QPainter p(this);
    style()->drawPrimitive(QStyle::PE_Widget, &opt, &p, this);
}
