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

#include <QString>
#include <cmath>
#include <QDateTime>
#include <Qmenu>
#include <QDebug>
#include <QPalette>
#include <QWidget>

#include "QGC.h"
#include "UASManager.h"
#include "SlugsStatusWidget.h"
#include "SlugsMAV.h"



SlugsStatusWidget::SlugsStatusWidget(QWidget *parent) :
    QWidget(parent),
    uas(0),
    startTime(0),
    timeout(false),
    iconIsRed(true),
    disconnected(true),
    timeRemaining(0),
    totalSpeed(0),
    alt(0),
    localFrame(false),
    globalFrameKnown(false),
    lowPowerModeEnabled(true),
    generalUpdateCount(0),
    fixQuality(0),
    m_ui(new Ui::slugsStatus)
{

    //chargeAviLevel(0),
    //chargeCriLevel(0),
    //load(0),
    //state("UNKNOWN"),
    //stateDesc(tr("Unknown state")),
    //mode("MAV_MODE_UNKNOWN"),
    /*
    setAviBatterySpecsAction(new QAction(tr("Set Avionic Battery Options"), this)),
    setCriBatterySpecsAction(new QAction(tr("Set Critical Battery Options"), this)),
    */

    m_ui->setupUi(this);

    connect(UASManager::instance(), SIGNAL(activeUASSet(UASInterface*)), this, SLOT(setUAS(UASInterface*)));
    if (UASManager::instance()->getActiveUAS())
    {
        setUAS(UASManager::instance()->getActiveUAS());
    }
}

SlugsStatusWidget::~SlugsStatusWidget() {
    if (refreshTimer) disconnect(refreshTimer, SIGNAL(timeout()), this, SLOT(refresh()));
    refreshTimer = NULL;
}


void SlugsStatusWidget::setUAS(UASInterface* uas) {

    if (this->uas) {

        // Setup communication
        //disconnect(this->uas, SIGNAL(batteryChanged(UASInterface*, double, double, double, int)), this, SLOT(updateBattery(UASInterface*, double, double, double, int)));
        disconnect(this->uas, SIGNAL(heartbeat(UASInterface*)), this, SLOT(receiveHeartbeat(UASInterface*)));
//        //disconnect(this->uas, SIGNAL(thrustChanged(UASInterface*, double)), this, SLOT(updateThrust(UASInterface*, double)));
//        //disconnect(this->uas, SIGNAL(localPositionChanged(UASInterface*,double,double,double,quint64)), this, SLOT(updateLocalPosition(UASInterface*,double,double,double,quint64)));
//        //disconnect(this->uas, SIGNAL(globalPositionChanged(UASInterface*,double,double,double,quint64)), this, SLOT(updateGlobalPosition(UASInterface*,double,double,double,quint64)));
//        disconnect(this->uas, SIGNAL(velocityChanged_NED(UASInterface*,double,double,double,quint64)), this, SLOT(updateSpeed(UASInterface*,double,double,double,quint64)));
//        //disconnect(this->uas, SIGNAL(statusChanged(UASInterface*,QString,QString)), this, SLOT(updateState(UASInterface*,QString,QString)));
//        //disconnect(this->uas, SIGNAL(modeChanged(int,QString,QString)), this, SLOT(updateMode(int,QString,QString)));
//        //disconnect(this->uas, SIGNAL(loadChanged(UASInterface*, double)), this, SLOT(updateLoad(UASInterface*, double)));
//        disconnect(this->uas, SIGNAL(heartbeatTimeout(bool, unsigned int)), this, SLOT(heartbeatTimeout(bool, unsigned int)));
//        disconnect(this->uas, SIGNAL(systemTypeSet(UASInterface*,uint)), this, SLOT(setSystemType(UASInterface*,uint)));
//        //disconnect(this->uas, SIGNAL(textMessageReceived(int,int,int,QString)), this, SLOT(showStatusText(int, int, int, QString)));
//        //disconnect(this->uas, SIGNAL(navModeChanged(int, int, QString)), this, SLOT(updateNavMode(int, int, QString)));

//        /*
//        disconnect(setAviBatterySpecsAction, SIGNAL(triggered()), this, SLOT(setAviBatterySpecs()));
//        disconnect(setCriBatterySpecsAction, SIGNAL(triggered()), this, SLOT(setCriBatterySpecs()));
//        */
//        // Name changes
//        disconnect(uas, SIGNAL(nameChanged(QString)), this, SLOT(updateName(QString)));

//        if(uas->getAutopilotType() == MAV_AUTOPILOT_SLUGS)
//            disconnect((SlugsMAV*)(this->uas), SIGNAL(gpsFixChanged(UASInterface*, int)), this, SLOT(updateGpsFix(UASInterface*,int)));

        // Heartbeat fade
        disconnect(refreshTimer, SIGNAL(timeout()), this, SLOT(refresh()));
        refreshTimer = NULL;

    }
    if (uas) {
//        // Setup communication
//        //connect(uas, SIGNAL(batteryChanged(UASInterface*, double, double, double, int)), this, SLOT(updateBattery(UASInterface*, double, double, double, int)));
        connect(uas, SIGNAL(heartbeat(UASInterface*)), this, SLOT(receiveHeartbeat(UASInterface*)));
//        //connect(uas, SIGNAL(thrustChanged(UASInterface*, double)), this, SLOT(updateThrust(UASInterface*, double)));
//        connect(uas, SIGNAL(localPositionChanged(UASInterface*,double,double,double,quint64)), this, SLOT(updateLocalPosition(UASInterface*,double,double,double,quint64)));
//        connect(uas, SIGNAL(globalPositionChanged(UASInterface*,double,double,double,quint64)), this, SLOT(updateGlobalPosition(UASInterface*,double,double,double,quint64)));
//        connect(uas, SIGNAL(velocityChanged_NED(UASInterface*,double,double,double,quint64)), this, SLOT(updateSpeed(UASInterface*,double,double,double,quint64)));
//        //connect(uas, SIGNAL(statusChanged(UASInterface*,QString,QString)), this, SLOT(updateState(UASInterface*,QString,QString)));
//        //connect(uas, SIGNAL(modeChanged(int,QString,QString)), this, SLOT(updateMode(int,QString,QString)));
//        //connect(uas, SIGNAL(loadChanged(UASInterface*, double)), this, SLOT(updateLoad(UASInterface*, double)));
//        connect(uas, SIGNAL(heartbeatTimeout(bool, unsigned int)), this, SLOT(heartbeatTimeout(bool, unsigned int)));
//        connect(uas, SIGNAL(systemTypeSet(UASInterface*,uint)), this, SLOT(setSystemType(UASInterface*,uint)));
//        //connect(uas, SIGNAL(textMessageReceived(int,int,int,QString)), this, SLOT(showStatusText(int, int, int, QString)));
//        //connect(uas, SIGNAL(navModeChanged(int, int, QString)), this, SLOT(updateNavMode(int, int, QString)));
//        /*
//        connect(setAviBatterySpecsAction, SIGNAL(triggered()), this, SLOT(setAviBatterySpecs()));
//        connect(setCriBatterySpecsAction, SIGNAL(triggered()), this, SLOT(setCriBatterySpecs()));
//        */
//        // Name changes
//        connect(uas, SIGNAL(nameChanged(QString)), this, SLOT(updateName(QString)));

//        if(uas->getAutopilotType() == MAV_AUTOPILOT_SLUGS)
//            connect((SlugsMAV*)(uas), SIGNAL(gpsFixChanged(UASInterface*, int)), this, SLOT(updateGpsFix(UASInterface*,int)));

        // Heartbeat fade
        refreshTimer = new QTimer(this);
        connect(refreshTimer, SIGNAL(timeout()), this, SLOT(refresh()));
        if (lowPowerModeEnabled)
        {
            refreshTimer->start(updateInterval*3);
        }
        else
        {
            refreshTimer->start(updateInterval);
        }

        // Style some elements by default to the UAS' color.
        heartbeatColor = Qt::green;
        QString colorstyle("QLabel { background-color: %1; }");
        m_ui->typeLabel->setStyleSheet(colorstyle.arg(heartbeatColor.name()));
        //updateActiveUAS(uas, false);

        // Set state and mode
        //updateMode(uas->getUASID(), uas->getShortMode(), "");
        //updateState(uas, uas->getShortState(), "");
        setSystemType(uas, uas->getSystemType());
    }

}


void SlugsStatusWidget::receiveHeartbeat(UASInterface* uas)
{
    heartbeatColor = uas->getColor();
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
    if (uas == this->uas)
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
        timeRemaining = seconds;
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
void SlugsStatusWidget::setCriBatterySpecs()
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

void SlugsStatusWidget::setAviBatterySpecs()
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
    if (generalUpdateCount == 4)
    {
#if (QGC_EVENTLOOP_DEBUG)
        // qDebug() << "EVENTLOOP:" << __FILE__ << __LINE__;
#endif
        generalUpdateCount = 0;
        //// qDebug() << "UPDATING EVERYTHING";
        // State
        //m_ui->stateLabel->setText(state);
        //m_ui->statusTextLabel->setText(stateDesc);

        // Battery
        //m_ui->batteryBar->setValue(static_cast<int>(this->chargeLevel));
        //m_ui->loadBar->setValue(static_cast<int>(this->load));
        //m_ui->thrustBar->setValue(this->thrust);

        // Altitude
        //m_ui->altitudeLabel->setText(QString("%1 m").arg(alt, 6, 'f', 1, '0'));


        // Speed
        //QString speed("%1 m/s");
        //m_ui->speedLabel->setText(speed.arg(totalSpeed, 4, 'f', 1, '0'));

        // Thrust
        //m_ui->thrustBar->setValue(thrust * 100);

        // Time Elapsed
        //QDateTime time = MG::TIME::msecToQDateTime(uas->getUptime());

        /*
        quint64 filterTime = uas->getUptime() / 1000;
        int sec = static_cast<int>(filterTime - static_cast<int>(filterTime / 60) * 60);
        int min = static_cast<int>(filterTime / 60);
        int hours = static_cast<int>(filterTime - min * 60 - sec);
        QString timeText;
        timeText = timeText.sprintf("%02d:%02d:%02d", hours, min, sec);
        m_ui->timeElapsedLabel->setText(timeText);
        */
    }
    generalUpdateCount++;

    if (timeout)
    {
        // CRITICAL CONDITION, NO HEARTBEAT
        disconnected = true;

        QColor warnColor;
        if (iconIsRed)
        {
            warnColor = Qt::red;
        }
        else
        {
            warnColor = Qt::darkRed;
            refreshTimer->setInterval(errorUpdateInterval);
            refreshTimer->start();
        }
        QString style = QString("SlugsStatusWidget {background-color: %1;}").arg(warnColor.name());
        this->setStyleSheet(style);
        iconIsRed = !iconIsRed;
    }
    else
    {
        // If we're not in low power mode, add the additional visual effect of
        // fading out the color of the heartbeat for this UAS.
        if (!lowPowerModeEnabled)
        {
            heartbeatColor = heartbeatColor.darker(110);
            QString colorstyle("QLabel {background-color: %1;}");
            m_ui->heartBeatLabel->setStyleSheet(colorstyle.arg(heartbeatColor.name()));
            refreshTimer->setInterval(updateInterval);
            refreshTimer->start();
        }
    }
} // refresh()
