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
    altitudeIsRed(false),
    speedIsRed(false),
    altitudeIsWarning(false),
    speedIsWarning(false),
    airSpeed(0),
    z(0),
    alt(0),
    localFrame(false),
    globalFrameKnown(false),
    lowPowerModeEnabled(true),
    fixQuality(0),
    generalUpdateCount(0),
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


        // Heartbeat fade
        disconnect(this->uas, SIGNAL(heartbeatTimeout(bool, unsigned int)), this, SLOT(heartbeatTimeout(bool, unsigned int)));
        disconnect(this->uas, SIGNAL(heartbeat(UASInterface*)), this, SLOT(receiveHeartbeat(UASInterface*)));

        // Velocity and altitude
        disconnect(this->uas, SIGNAL(localPositionChanged(UASInterface*,double,double,double,quint64)), this, SLOT(updateLocalPosition(UASInterface*,double,double,double,quint64)));
        disconnect(this->uas, SIGNAL(globalPositionChanged(UASInterface*,double,double,double,quint64)), this, SLOT(updateGlobalPosition(UASInterface*,double,double,double,quint64)));
        //disconnect(this->uas, SIGNAL(velocityChanged_NED(UASInterface*,double,double,double,quint64)), this, SLOT(updateSpeed(UASInterface*,double,double,double,quint64)));

        // System type and battery related
        //disconnect(this->uas, SIGNAL(systemTypeSet(UASInterface*,uint)), this, SLOT(setSystemType(UASInterface*,uint)));
        disconnect(this->uas, SIGNAL(batteryChanged(UASInterface*, double, double, double, int)), this, SLOT(updateBattery(UASInterface*, double, double, double, int)));
        disconnect(setBatterySpecsAction, SIGNAL(triggered()), this, SLOT(setBatterySpecs()));

        // Name changes
        disconnect(uas, SIGNAL(nameChanged(QString)), this, SLOT(updateName(QString)));
        m_ui->groupBox->setTitle("");

        // Slugs only
        if(this->uas->getAutopilotType() == MAV_AUTOPILOT_SLUGS) {
            disconnect((SlugsMAV*)(this->uas), SIGNAL(gpsFixChanged(int, unsigned int)), this, SLOT(updateGpsFix(int, unsigned int)));
            disconnect((SlugsMAV*)(this->uas), SIGNAL(loadsChanged(int, double, double)), this, SLOT(updateLoad(int, double, double)));
            disconnect((SlugsMAV*)(this->uas), SIGNAL(airSpeedChanged(int, float)), this, SLOT(updateSpeed(int, float)));
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
        connect(uas, SIGNAL(localPositionChanged(UASInterface*,double,double,double,quint64)), this, SLOT(updateLocalPosition(UASInterface*,double,double,double,quint64)));
        connect(uas, SIGNAL(globalPositionChanged(UASInterface*,double,double,double,quint64)), this, SLOT(updateGlobalPosition(UASInterface*,double,double,double,quint64)));
        //connect(uas, SIGNAL(velocityChanged_NED(UASInterface*,double,double,double,quint64)), this, SLOT(updateSpeed(UASInterface*,double,double,double,quint64)));

        // System type and battery related
        //connect(uas, SIGNAL(systemTypeSet(UASInterface*,uint)), this, SLOT(setSystemType(UASInterface*,uint)));
        connect(uas, SIGNAL(batteryChanged(UASInterface*, double, double, double, int)), this, SLOT(updateBattery(UASInterface*, double, double, double, int)));
        connect(setBatterySpecsAction, SIGNAL(triggered()), this, SLOT(setBatterySpecs()));

        // Name changes
        connect(uas, SIGNAL(nameChanged(QString)), this, SLOT(updateName(QString)));
        updateName("");

        // Slugs only
        if(uas->getAutopilotType() == MAV_AUTOPILOT_SLUGS) {
            connect((SlugsMAV*)(uas), SIGNAL(gpsFixChanged(int, unsigned int)), this, SLOT(updateGpsFix(int, unsigned int)));
            connect((SlugsMAV*)(uas), SIGNAL(loadsChanged(int, double, double)), this, SLOT(updateLoad(int, double, double)));
            connect((SlugsMAV*)(uas), SIGNAL(airSpeedChanged(int, float)), this, SLOT(updateSpeed(int, float)));
        }

        // Heartbeat fade
        connect(uas, SIGNAL(heartbeat(UASInterface*)), this, SLOT(receiveHeartbeat(UASInterface*)));
        connect(uas, SIGNAL(heartbeatTimeout(bool, unsigned int)), this, SLOT(heartbeatTimeout(bool, unsigned int)));
        refreshTimer->setInterval( (lowPowerModeEnabled)? updateIntervalLowPower : updateInterval);
        refreshTimer->start();

        // Style heartbeat and gps indicators
        heartbeatColor = Qt::green;
        QString colorstyle("QLabel { background-color: %1; }");
        //m_ui->typeLabel->setStyleSheet(colorstyle.arg(heartbeatColor.name()));

        //setSystemType(uas, uas->getSystemType());
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


void SlugsStatusWidget::updateName(const QString& name)
{
    Q_UNUSED(name);

    if (!uas) {
        m_ui->groupBox->setTitle("");
        return;
    }

    if (uas->getUASName() == "")
    {
        m_ui->groupBox->setTitle(tr("UAS") + QString::number(uas->getUASID()));
    }
    else
    {
        m_ui->groupBox->setTitle(uas->getUASName());
    }
}


/**
 * The current system type is represented through the system icon.
 *
 * @param uas Source system, has to be the same as this->uas
 * @param systemType type ID, following the MAVLink system type conventions
 * @see http://pixhawk.ethz.ch/software/mavlink
 */
/*
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
*/

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

void SlugsStatusWidget::updateSpeed(int id, float airSpeed)
{
    if (uas && (uas->getUASID() == id))
    {
        this->airSpeed = airSpeed;
    }
}

void SlugsStatusWidget::updateBattery(UASInterface* uas, double voltage, double current, double percent, int seconds)
{
    Q_UNUSED(voltage);
    Q_UNUSED(current);
    Q_UNUSED(seconds);
    if (this->uas == uas)
    {
        //timeRemaining = seconds;
        chargeLevel = percent;
    }
}


void SlugsStatusWidget::updateLoad(int id, double ctrlLoad, double sensLoad)
{
    if (uas && (uas->getUASID() == id))
    {
        this->ctrlLoad = ctrlLoad;
        this->sensLoad = sensLoad;
    }
}

void SlugsStatusWidget::updateGpsFix(int id, unsigned int fixQuality) {
    if (uas && (uas->getUASID() == id))
    {
        this->fixQuality = fixQuality;

        gpsColor = (fixQuality > 0)? Qt::green : Qt::red;
        QString colorstyle("QLabel { background-color: %1; }");
        m_ui->gpsStatusLabel->setStyleSheet(colorstyle.arg(gpsColor.name()));
    }
}

void SlugsStatusWidget::contextMenuEvent (QContextMenuEvent* event)
{
    QMenu menu(this);
    menu.addAction(setBatterySpecsAction);
    menu.exec(event->globalPos());
}

void SlugsStatusWidget::showEvent(QShowEvent* event)
{
    // React only to internal (pre-display)
    // events
    Q_UNUSED(event);
    refreshTimer->start(updateInterval*10);
}

void SlugsStatusWidget::hideEvent(QHideEvent* event)
{
    // React only to internal (pre-display)
    // events
    Q_UNUSED(event);
   // refreshTimer->stop();
}

void SlugsStatusWidget::changeEvent(QEvent *e)
{
    QWidget::changeEvent(e);
    switch (e->type())
    {
    case QEvent::LanguageChange:
        m_ui->retranslateUi(this);
        break;
    default:
        break;
    }
}


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


void SlugsStatusWidget::refresh()
{
    if (!disconnected && generalUpdateCount == updateDisplayIntervalMultiplier && uas)
    {
        // Update interface at 1/4th of speed
        generalUpdateCount = 0;

        // GPS fix
        //m_ui->gpsStatusLabel

        // Battery
        m_ui->batteryBar->setValue(static_cast<int>(chargeLevel));
        QString batteryTipText = QString("%1 V Battery Charge").arg(chargeLevel, 1, 'f', 1);
        m_ui->batteryBar->setToolTip(batteryTipText);
        m_ui->batteryBar->setStatusTip(batteryTipText);

        m_ui->loadCtrlBar->setValue(static_cast<int>(this->ctrlLoad));
        QString loadCtrlTipText = QString("%1\% Control DSC Load").arg(static_cast<int>(this->ctrlLoad));
        m_ui->loadCtrlBar->setToolTip(loadCtrlTipText);
        m_ui->loadCtrlBar->setStatusTip(loadCtrlTipText);

        m_ui->loadSensBar->setValue(static_cast<int>(this->sensLoad));
        QString loadSensTipText = QString("%1\% Sensor DSC Load").arg(static_cast<int>(this->sensLoad));
        m_ui->loadSensBar->setToolTip(loadSensTipText);
        m_ui->loadSensBar->setStatusTip(loadSensTipText);

        // Altitude
        // TODO make warning limits configurable during runtime
        float altitude = (globalFrameKnown)? this->alt : this->z;
        int altitudePrecision = (altitude >= 100.0)? 1 : 2;
        QString altitudeText = QString("%1").arg(altitude, 3, 'f', altitudePrecision);
        QString altitudeTipText = QString("Altitude: %1 m").arg(altitude, 1, 'f', 2);
        m_ui->altitudeLabel->setText(altitudeText);
        m_ui->altitudeLabel->setToolTip(altitudeTipText);
        m_ui->altitudeLabel->setStatusTip(altitudeTipText);

        // altitude coloring (red beyond limits, yellow if close to limits, and green otherwise)
        QColor altitudeColor = Qt::green;
        if (altitude <= altitudeLowLimit || altitude >= altitudeHighLimit) {
            altitudeColor = (altitudeIsRed)? Qt::green : Qt::red;
            altitudeIsWarning = true;
        }
        else {
            altitudeIsWarning = false;
        }
        if ((altitude > altitudeLowLimit && altitude <= (altitudeLowLimit + altitudeLimitThreshold))
            || (altitude < altitudeHighLimit && altitude >= (altitudeHighLimit + altitudeLimitThreshold)))
            altitudeColor = QColor(255, 127, 0); // orange
        QString altitudeStyle = QString("QLabel {background-color: %1;}").arg(altitudeColor.name());
        m_ui->altitudeLabel->setStyleSheet(altitudeStyle);
        altitudeIsRed = altitudeColor == Qt::red;

        // Airspeed
        // TODO make warning limits configurable during runtime
        int speedPrecision = (airSpeed >= 100.0)? 1 : 2;
        QString speedText = QString("%1").arg(airSpeed, 3, 'f',speedPrecision);
        QString speedTipText = QString("Airspeed: %1 m/s").arg(airSpeed, 1, 'f', 2);
        m_ui->speedLabel->setText(speedText);
        m_ui->speedLabel->setToolTip(speedTipText);
        m_ui->speedLabel->setStatusTip(speedTipText);

        // airSpeed coloring (red beyond limits, yellow if close to limits, and green otherwise)
        QColor speedColor = Qt::green;
        if (airSpeed <= airSpeedLowLimit || airSpeed >= airSpeedHighLimit) {
            speedColor = (speedIsRed)? Qt::green : Qt::red;
            speedIsWarning = true;
        }
        else {
            speedIsWarning = false;
        }
        if ((airSpeed > airSpeedLowLimit && airSpeed <= (airSpeedLowLimit + airSpeedLimitThreshold))
            || (airSpeed < airSpeedHighLimit && airSpeed >= (airSpeedHighLimit + airSpeedLimitThreshold)))
            speedColor =  QColor(255, 127, 0); // orange
        QString speedStyle = QString("QLabel {background-color: %1;}").arg(speedColor.name());
        m_ui->speedLabel->setStyleSheet(speedStyle);
        speedIsRed = speedColor == Qt::red;

    }
    else {

        if (speedIsWarning && !speedIsRed) {
            QColor speedColor = Qt::red;
            QString speedStyle = QString("QLabel {background-color: %1;}").arg(speedColor.name());
            m_ui->speedLabel->setStyleSheet(speedStyle);
        }

        if (altitudeIsWarning && !altitudeIsRed) {
            QColor altitudeColor = Qt::red;
            QString altitudeStyle = QString("QLabel {background-color: %1;}").arg(altitudeColor.name());
            m_ui->altitudeLabel->setStyleSheet(altitudeStyle);
        }
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
        heartbeatColor = heartbeatColor.darker(135);
        QString colorstyle("QLabel {background-color: %1;}");
        m_ui->heartBeatLabel->setStyleSheet(colorstyle.arg(heartbeatColor.name()));
        refreshTimer->setInterval( (lowPowerModeEnabled)? updateIntervalLowPower : updateInterval);
        refreshTimer->start();
        //}
    }

    // TODO set status background to red
    //QColor typeColor = (disconnected)? Qt::red : Qt::green;
    //QString typeStyle = QString("QLabel {background-color: %1;}").arg(typeColor.name());
    //m_ui->typeLabel->setStyleSheet(typeStyle);
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
