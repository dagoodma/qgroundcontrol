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
    gpsFixQuality(0),
    generalUpdateCount(0),
    batteryCharge(0),
    batteryVoltage(0),
    ctrlLoad(0),
    sensLoad(0),
    setBatterySpecsAction(new QAction(tr("Set Battery Options"), this)),
    airSpeedLowLimit(12),
    airSpeedHighLimit(25),
    airSpeedLimitThreshold(1.5),
    altitudeLowLimit(50),
    altitudeLimitThreshold(25),
    altitudeHighLimit(250),
    m_ui(new Ui::slugsStatus),
    isReturning(false),
    link(0)
{

    m_ui->setupUi(this);

    refreshTimer = new QTimer(this);
    connect(refreshTimer, SIGNAL(timeout()), this, SLOT(refresh()));
    setUAS(UASManager::instance()->getActiveUAS());
    connect(UASManager::instance(), SIGNAL(activeUASSet(UASInterface*)), this, SLOT(setUAS(UASInterface*)));
}

SlugsStatusWidget::~SlugsStatusWidget() {
    //if (refreshTimer) disconnect(refreshTimer, SIGNAL(timeout()), this, SLOT(refresh()));
    //disconnect(refreshTimer, SIGNAL(timeout()), this, SLOT(refresh()));
    //setUAS(NULL);


    //delete m_ui;
}


void SlugsStatusWidget::setUAS(UASInterface* uas) {

    // Do nothing if system is the same
    if (this->uas == uas || uas == NULL)
        return;

    if (this->uas) {
        qDebug() << "Disconnecting old UAS in status widget.";
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

        // Link
        link = 0;

    }

    this->uas = uas;
    if (uas) {
        // Setup communication
        qDebug() << "Setting new UAS in status widget.";
        disconnected = false;

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

        // Link
        QList<LinkInterface*>* x = uas->getLinks();
        if (x->size())
        {
            LinkInterface* li = x->at(0);
            link = li;
        }

    } // uas
    else {

        // Disconnect and stop timer
        disconnected = true;
        refresh();
        refreshTimer->stop();
        qDebug() << "UAS disconnected.";
    }


}


void SlugsStatusWidget::heartbeatTimeout(bool timeout, unsigned int ms)
{
    Q_UNUSED(ms);
    this->timeout = timeout;
}


void SlugsStatusWidget::receiveHeartbeat(UASInterface* uas)
{
    //heartbeatColor = uas->getColor();
    heartbeatColor = Qt::green;


    // Heartbeat from groundstation (set to orange if returning to base)
    if (uas->getAutopilotType() == MAV_AUTOPILOT_SLUGS) {
        SlugsMAV* mav = (SlugsMAV*)uas;
        isReturning = mav->IsReturning();
    }

    heartbeatColor = (isReturning)? QColor(255,115,25) : Qt::green;
    QString colorstyle("QLabel { background-color: %1; }");
    m_ui->heartBeatLabel->setStyleSheet(colorstyle.arg(heartbeatColor.name()));


    disconnected = false;
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
    Q_UNUSED(current);
    Q_UNUSED(seconds);
    if (this->uas == uas)
    {
        //timeRemaining = seconds;
        batteryCharge = percent;
        batteryVoltage = voltage;
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

void SlugsStatusWidget::updateGpsFix(int id, unsigned int gpsFixQuality) {
    if (uas && (uas->getUASID() == id))
    {
        this->gpsFixQuality = gpsFixQuality;

        gpsColor = (gpsFixQuality > 0)? Qt::green : Qt::red;
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

    if (!disconnected && generalUpdateCount >= updateDisplayIntervalMultiplier && uas)
    {
        // Update interface at 1/4th of speed
        generalUpdateCount = 0;

        // Heartbeat fading
        heartbeatColor = heartbeatColor.darker(135);
        QString colorstyle("QLabel {background-color: %1;}");
        m_ui->heartBeatLabel->setStyleSheet(colorstyle.arg(heartbeatColor.name()));

        // Battery
        m_ui->batteryBar->setValue(static_cast<int>(batteryCharge));
        QString batteryTipText = QString("%1 V Battery Charge").arg(batteryVoltage, 1, 'f', 1);
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
        altitudeIsWarning = (altitude <= altitudeLowLimit || altitude >= altitudeHighLimit);

        if ((altitude > altitudeLowLimit && altitude <= (altitudeLowLimit + altitudeLimitThreshold))
            || (altitude < altitudeHighLimit && altitude >= (altitudeHighLimit + altitudeLimitThreshold)))
            altitudeColor = QColor(255, 127, 0); //
        if (!altitudeIsWarning) {
            QString altitudeStyle = QString("QLabel {background-color: %1;}").arg(altitudeColor.name());
            m_ui->altitudeLabel->setStyleSheet(altitudeStyle);
        }

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
        speedIsWarning = (airSpeed <= airSpeedLowLimit || airSpeed >= airSpeedHighLimit);

        if ((airSpeed > airSpeedLowLimit && airSpeed <= (airSpeedLowLimit + airSpeedLimitThreshold))
            || (airSpeed < airSpeedHighLimit && airSpeed >= (airSpeedHighLimit + airSpeedLimitThreshold)))
            speedColor =  QColor(255, 127, 0); // orange
        if (!speedIsWarning) {
            QString speedStyle = QString("QLabel {background-color: %1;}").arg(speedColor.name());
            m_ui->speedLabel->setStyleSheet(speedStyle);
        }

    }
    generalUpdateCount++;

    // Gps status

    // GPS fix (todo: get string names for fix types)
    // set the color
    gpsColor = (gpsFixQuality > 0)? Qt::green : Qt::red;
    QString colorstyle("QLabel { background-color: %1; }");
    m_ui->gpsStatusLabel->setStyleSheet(colorstyle.arg(gpsColor.name()));
    // set the tool tip text
    QString gpsLabelText = QString("GPS fix quality: %1").arg(gpsFixQuality);
    m_ui->gpsStatusLabel->setToolTip(gpsLabelText);
    m_ui->gpsStatusLabel->setStatusTip(gpsLabelText);


    // Speed warning indicator blink
    if (speedIsWarning) {
        QColor speedColor = (speedIsRed)? Qt::green : Qt::red;
        QString speedStyle = QString("QLabel {background-color: %1;}").arg(speedColor.name());
        m_ui->speedLabel->setStyleSheet(speedStyle);
        speedIsRed = speedColor == Qt::red;
    }
    // Altitude warning indicator blink
    if(altitudeIsWarning) {
        QColor altitudeColor = (altitudeIsRed)? Qt::green : Qt::red;
        QString altitudeStyle = QString("QLabel {background-color: %1;}").arg(altitudeColor.name());
        m_ui->altitudeLabel->setStyleSheet(altitudeStyle);
        altitudeIsRed = altitudeColor == Qt::red;
    }

    // Heartbeat timeout check
    if (timeout)
    {
        // CRITICAL CONDITION, NO HEARTBEAT
        disconnected = true;

        QColor warnColor = Qt::red;
        QString style = QString("QLabel {background-color: %1;}").arg(warnColor.name());
        m_ui->heartBeatLabel->setStyleSheet(style);

        QString heartbeatStatusText("<b>MAV communication</b>: no connection");
        m_ui->heartBeatLabel->setToolTip(heartbeatStatusText);
        m_ui->heartBeatLabel->setStatusTip(heartbeatStatusText);

        refreshTimer->setInterval(errorUpdateInterval);
        refreshTimer->start();
    }
    else
    {
        refreshTimer->setInterval( (lowPowerModeEnabled)? updateIntervalLowPower : updateInterval);
        refreshTimer->start();

        QString heartbeatStatusText("<b>MAV communication</b>: %1");
        if (isReturning) {
            heartbeatStatusText = heartbeatStatusText.arg("lost ground station heartbeat. Ensure heartbeat is emitted and then set navigation mode.");
        }
        else {
            heartbeatStatusText = heartbeatStatusText.arg("is working.");
        }

        // Link quality
        if (link)
        {
            ProtocolInterface* p = LinkManager::instance()->getProtocolForLink(link);

            // Build the tooltip out of the protocol parsing data: received, dropped, and parsing errors.
            QString displayString("");
            int c;
            if ((c = p->getReceivedPacketCount(link)) != -1)
            {
                displayString += QString(tr("<br/>Received: %2")).arg(QString::number(c));
            }
            if ((c = p->getDroppedPacketCount(link)) != -1)
            {
                displayString += QString(tr("<br/>Dropped: %2")).arg(QString::number(c));
            }
            if ((c = p->getParsingErrorCount(link)) != -1)
            {
                displayString += QString(tr("<br/>Errors: %2")).arg(QString::number(c));
            }
            if (!displayString.isEmpty())
            {
                heartbeatStatusText = QString("%1<br/><br/><b>%2</b>").arg(heartbeatStatusText).arg(link->getName())
                        + displayString;
            }
        }
        m_ui->heartBeatLabel->setToolTip(heartbeatStatusText);
        m_ui->heartBeatLabel->setStatusTip(heartbeatStatusText);

    }
}


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
