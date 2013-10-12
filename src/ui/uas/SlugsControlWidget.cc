/*=====================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit

(c) 2009, 2010 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

This file is part of the PIXHAWK project

    PIXHAWK is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    PIXHAWK is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with PIXHAWK. If not, see <http://www.gnu.org/licenses/>.

======================================================================*/

/**
 * @file
 *   @brief Definition of widget controlling a SLUGS MAV
 *
 *   @author David Goodman <dagoodman@soe.ucsc.edu>
 *
 */

#include <QString>
//#include <QTimer>
#include <QDebug>
#include <QLabel>
//#include <QFileDialog>
#include <QDebug>
#include <QMap>
#include <QPalette>

#include "MainWindow.h"

#include "SlugsControlWidget.h"

SlugsControlWidget::SlugsControlWidget(QWidget *parent) :
    QWidget(parent),
    //mainWindow((MainWindow*)(parent->parent())),
    uas(0),
    uasNavigationModeSent(0),
    uasNavigationMode(0),
    uasMidLevelRequestSent(false)
    //engineOn(false)
{
    ui.setupUi(this);

    ui.linePatrolModeButton->hide();

    // Arm/disarm
    connect(ui.armDisarmButton,SIGNAL(clicked()),this,SLOT(armDisarmButtonClicked()));

    // Mid-level commands tab
    connect(ui.getMidLevelButton, SIGNAL(clicked()), this, SLOT(getMidLevelButtonClicked()));
    connect(ui.setMidLevelButton, SIGNAL(clicked()), this, SLOT(setMidLevelButtonClicked()));
    connect(ui.writeMidLevelButton, SIGNAL(clicked()), this, SLOT(writeMidLevelButtonClicked()));
    connect(ui.readMidLevelButton, SIGNAL(clicked()), this, SLOT(readMidLevelButtonClicked()));

    // Selective passthrough control surfaces tab
    connect(ui.setPassthroughButton, SIGNAL(clicked()), this, SLOT(setPassthroughButtonClicked()));

    // Navigation modes panel
#ifdef MAVLINK_ENABLED_SLUGS
    connect(ui.waypointModeButton, SIGNAL(clicked()), this, SLOT(waypointModeButtonClicked()));
    connect(ui.isrModeButton, SIGNAL(clicked()), this, SLOT(isrModeButtonClicked()));
    connect(ui.selectivePassthroughModeButton, SIGNAL(clicked()), this, SLOT(selectivePassthroughModeButtonClicked()));
    connect(ui.passthroughModeButton, SIGNAL(clicked()), this, SLOT(passthroughModeButtonClicked()));
    connect(ui.midLevelModeButton, SIGNAL(clicked()), this, SLOT(midLevelModeButtonClicked()));
    connect(ui.linePatrolModeButton, SIGNAL(clicked()), this, SLOT(linePatrolModeButtonClicked()));

    navModeButtons[SLUGS_MODE_WAYPOINT] = ui.waypointModeButton;
    navModeButtons[SLUGS_MODE_ISR] = ui.isrModeButton;
    navModeButtons[SLUGS_MODE_SELECTIVE_PASSTHROUGH] = ui.selectivePassthroughModeButton;
    navModeButtons[SLUGS_MODE_PASSTHROUGH] = ui.passthroughModeButton;
    navModeButtons[SLUGS_MODE_MID_LEVEL] = ui.midLevelModeButton;
    navModeButtons[SLUGS_MODE_LINE_PATROL] = ui.linePatrolModeButton;
#endif
    connect(UASManager::instance(), SIGNAL(activeUASSet(UASInterface*)), this, SLOT(setUAS(UASInterface*)));
    if (UASManager::instance()->getActiveUAS())
    {
        setUAS(UASManager::instance()->getActiveUAS());
    }
}


SlugsControlWidget::~SlugsControlWidget()
{

}

/**
 * @param uas To control.
 */
void SlugsControlWidget::setUAS(UASInterface* uas)
{
    if (this->uas)
    {
        UASInterface* oldUAS = UASManager::instance()->getUASForId(this->uas);

        // UAS signals
        disconnect(oldUAS, SIGNAL(midLevelCommandsChanged(int,double,double,double)), this, SLOT(updateMidLevelParameters(int,double,double,double)));
        disconnect(oldUAS,SIGNAL(navModeChanged(int,int,QString)), this, SLOT(updateNavMode(int,int,QString)));
        disconnect(oldUAS,SIGNAL(armingChanged(bool)),this,SLOT(updateArmDisarm(bool)));

        //ui.controlStatusLabel->setText(tr("UNCONNECTED"));
    }

    // Connect user interface controls
    if (uas)
    {

        // UAS signals
        connect(uas, SIGNAL(midLevelCommandsChanged(int,double,double,double)), this, SLOT(updateMidLevelParameters(int,double,double,double)));
        connect(uas, SIGNAL(navModeChanged(int,int,QString)), this, SLOT(updateNavMode(int,int,QString)));
        connect(uas,SIGNAL(armingChanged(bool)),this,SLOT(updateArmDisarm(bool)));

        // Select the current navigation mode button
        if (uas->getAutopilotType() == MAV_AUTOPILOT_SLUGS) {
            SlugsMAV *slugsUas = static_cast<SlugsMAV*>(uas);

            int navigationMode = slugsUas->getNavMode();
            updateNavMode(uas->getUASID(), navigationMode, slugsUas->getNavModeText(navigationMode));
        }


        //ui.controlStatusLabel->setText(tr("Connected to ") + uas->getUASName());

        this->uas = uas->getUASID();

        updateArmDisarm(uas->isArmed());
        //setBackgroundColor(uas->getColor());
    }
    else
    {
        this->uas = -1;
    }
}

void SlugsControlWidget::armDisarmButtonClicked()
{
    if (uas)
    {
        UASInterface *mav = UASManager::instance()->getUASForId(this->uas);
        if (!mav) return;

        if (mav->isArmed())
        {
            ((UAS*)mav)->disarmSystem();
        }
        else
        {
            ((UAS*)mav)->armSystem();
        }

        //waitingToArmDisarm = true; // TODO only change last action if clicked
    }
}

/**
 * @param state Arm/disarm state (armed = true, disarmed = false).
 */
void SlugsControlWidget::updateArmDisarm(bool state)
{
    if (!uas) return;

    //TODO:
    //Figure out why arm/disarm is in UAS.h and not part of the interface, and fix.
    if (state)
    {
        ui.armDisarmButton->setText("DISARM");
        ui.armDisarmButton->setToolTip("Disarm the MAV (currently armed).");
        ui.lastActionLabel->setText(QString("Armed motors"));
    }
    else
    {
        ui.armDisarmButton->setText("ARM");
        ui.armDisarmButton->setToolTip("Arm the MAV (currently disarmed).");
        ui.lastActionLabel->setText(QString("Disarmed motors"));
    }
}


void SlugsControlWidget::updateMidLevelParameters(int uas, double altitude, double airspeed, double turnrate) {
    if (!uas || uas != this->uas) return;

    ui.altitudeSpinBox->setValue(altitude);
    ui.airspeedSpinBox->setValue(airspeed);
    ui.turnRateSpinBox->setValue(turnrate);

    // TODO only update action label if get button was clicked
    //ui.lastActionLabel->setText(QString("Read new mid-level commands from %1").arg(description));
    if (uasMidLevelRequestSent)
        ui.lastActionLabel->setText(QString("Read new mid-level commands"));

    uasMidLevelRequestSent = false;
}


void SlugsControlWidget::getMidLevelButtonClicked() {
#ifdef MAVLINK_ENABLED_SLUGS
    if (uas) {
        UASInterface* mav = UASManager::instance()->getUASForId(this->uas);
        if (!mav) return;


        if (mav->getAutopilotType() != MAV_AUTOPILOT_SLUGS) {
            // TODO show error popup
           return;
        }

        SlugsMAV* slugsMav = static_cast<SlugsMAV*>(mav);
        slugsMav->requestMidLevelCommands();

        uasMidLevelRequestSent = true;
        ui.lastActionLabel->setText(QString("Requested mid-level commands"));

    }
#endif
}


void SlugsControlWidget::setMidLevelButtonClicked() {
#ifdef MAVLINK_ENABLED_SLUGS
    if (uas) {
        UASInterface* mav = UASManager::instance()->getUASForId(this->uas);
        if (!mav) return;

        if (mav->getAutopilotType() != MAV_AUTOPILOT_SLUGS) {
            // TODO show error popup
           return;
        }

        SlugsMAV* slugsMav = static_cast<SlugsMAV*>(mav);
        slugsMav->setMidLevelCommands(ui.altitudeSpinBox->value(),ui.airspeedSpinBox->value(),ui.turnRateSpinBox->value());

        ui.lastActionLabel->setText(QString("Sent new mid-level commands"));
        uasMidLevelRequestSent = false;
    }
#endif
}

void SlugsControlWidget::writeMidLevelButtonClicked() {
#ifdef MAVLINK_ENABLED_SLUGS
    if (!uas) return;

    UASInterface* mav = UASManager::instance()->getUASForId(this->uas);
    if (!mav) return;

    SlugsMAV* slugsMav = static_cast<SlugsMAV*>(mav);

    slugsMav->writeMidLevelCommandsToEeprom();

    ui.lastActionLabel->setText(QString("Write mid-level commands to EEPROM"));
#endif
}

void SlugsControlWidget::readMidLevelButtonClicked() {
#ifdef MAVLINK_ENABLED_SLUGS
    if (!uas) return;

    UASInterface* mav = UASManager::instance()->getUASForId(this->uas);
    if (!mav) return;

    SlugsMAV* slugsMav = static_cast<SlugsMAV*>(mav);

    slugsMav->readMidLevelCommandsFromEeprom();
    ui.lastActionLabel->setText(QString("Read mid-level commands from EEPROM"));
#endif
}

void SlugsControlWidget::setPassthroughButtonClicked() {
#ifdef MAVLINK_ENABLED_SLUGS
    if (uas) {
        UASInterface* mav = UASManager::instance()->getUASForId(this->uas);
        if (!mav) return;

        if (mav->getAutopilotType() != MAV_AUTOPILOT_SLUGS) {
            // TODO show error popup
           return;
        }

        SlugsMAV* slugsMav = static_cast<SlugsMAV*>(mav);
        slugsMav->setPassthroughSurfaces(ui.throttleCheckBox->isChecked(),
                                         ui.aileronsCheckBox->isChecked(),
                                         ui.rudderCheckBox->isChecked(),
                                         ui.elevatorCheckBox->isChecked());

        ui.lastActionLabel->setText(QString("Sent passthrough selecton"));

    }
#endif
}


/**
 * Set the background color based on the MAV color. If the MAV is selected as the
 * currently actively controlled system, the frame color is highlighted
 */
/*
void SlugsControlWidget::setBackgroundColor(QColor color)
{
    // UAS color
    QColor uasColor = color;
    QString colorstyle;
    QString borderColor = "#4A4A4F";
    borderColor = "#FA4A4F";
    uasColor = uasColor.darker(900);
    colorstyle = colorstyle.sprintf("QLabel { border-radius: 3px; padding: 0px; margin: 0px; background-color: #%02X%02X%02X; border: 0px solid %s; }",
                                    uasColor.red(), uasColor.green(), uasColor.blue(), borderColor.toStdString().c_str());
    setStyleSheet(colorstyle);
    QPalette palette = this->palette();
    palette.setBrush(QPalette::Window, QBrush(uasColor));
    setPalette(palette);
    setAutoFillBackground(true);
}
*/

/**
 * @param uas The ID of the UAS that changed navigation modes.
 * @param mode The new navigation mode.
 * @param description Text describing the new navigation mode.
 */
void SlugsControlWidget::updateNavMode(int uas, int mode,QString description)
{
    if (uas != this->uas) return;

    // TODO Show the button for the current navigation mode as selected
    // TODO only change action label if button was clicked
#ifdef MAVLINK_ENABLED_SLUGS
    if (mode == uasNavigationModeSent)
        ui.lastActionLabel->setText(QString("Set navigation mode to %1").arg(description));


    UASInterface* mav = UASManager::instance()->getUASForId(this->uas);
    if (!mav || mav->getAutopilotType() != MAV_AUTOPILOT_SLUGS) {
        // TODO show error popup
       return;
    }

    SlugsMAV* slugsMav = static_cast<SlugsMAV*>(mav);
    if (slugsMav->IsReturning()) {
        // TODO flash something on the status widget (make all buttons blink)
        qDebug() << QString("MAV lost ground station heartbeat and is returning home.");
        // deselect the button that was pressed
        if (navModeButtons.contains(uasNavigationMode)) {
            navModeButtons.value(uasNavigationMode)->setStyleSheet("");
        }
    }
    else if (navModeButtons.contains(mode) ) {
        QString buttonText = ((QPushButton*)navModeButtons.value(mode))->text();
        qDebug() << QString("Nav mode button %1 was pressed: %2").arg(mode).arg(buttonText);


        if (navModeButtons.contains(uasNavigationMode))
            qDebug() << QString("The old class is: %1").arg(navModeButtons.value(uasNavigationMode)->metaObject()->className());

        MainWindow::QGC_MAINWINDOW_STYLE style = MainWindow::instance()->getStyle();
        //QString buttonPressedStyle = "QPushButton { text-shadow: -1px -1px 1px #fff, 1px 1px 1px #000; }";

        //MainWindow::QGC_MAINWINDOW_STYLE style = mainWindow->getStyle();
        QString pressedStyle = (style == MainWindow::QGC_MAINWINDOW_STYLE_DARK)?
                    "QPushButton { color: #000; background-color: #fff; }"
                  : "QPushButton { color: #fff; background-color: #000; }"; //QColor(80,80,80) : QColor(40,40,40);

        //QColor pressedColor = QColor(65,65,80);
        //QString buttonPressedStyle = "QPushButton { background-color: %1; }";
        if (navModeButtons.contains(uasNavigationMode)) {
            navModeButtons.value(uasNavigationMode)->setStyleSheet("");
        }
        navModeButtons.value(mode)->setStyleSheet(pressedStyle);

    } // else if (navModeButtons.contains(mode) )

#endif
    uasNavigationMode = mode;
    uasNavigationModeSent = 0;
}

/*
void SlugsControlWidget::updateState(int state)
{
    switch (state)
    {
    case (int)MAV_STATE_ACTIVE:
        engineOn = true;
        ui.controlButton->setText(tr("DISARM SYSTEM"));
        break;
    case (int)MAV_STATE_STANDBY:
        engineOn = false;
        ui.controlButton->setText(tr("ARM SYSTEM"));
        break;
    }
}
*/

/*
void SlugsControlWidget::transmitMode()
{
    UASInterface* mav = UASManager::instance()->getUASForId(this->uas);
    if (mav)
    {
        // include armed state
        if (engineOn)
            uasMode |= MAV_MODE_FLAG_SAFETY_ARMED;
        else
            uasMode &= ~MAV_MODE_FLAG_SAFETY_ARMED;

        mav->setMode(uasMode);
        QString mode = ui.modeComboBox->currentText();

        ui.lastActionLabel->setText(QString("Sent new mode %1 to %2").arg(mode).arg(mav->getUASName()));
    }
}
*/

/**
 * @param mode Navigation mode corresponding to the button pressed.
 */
void SlugsControlWidget::navigationModeButtonClicked(int mode)
{
    if (uas) {
        // Adapt context button mode
        uasNavigationModeSent = mode;

        transmitNavigationMode(mode);

        // TODO start timer to clear uasNavigationMode after time limit?
    }
}

/**
 * @param mode New navigation mode to transmit to the MAV.
 */
void SlugsControlWidget::transmitNavigationMode(int mode)
{
#ifdef MAVLINK_ENABLED_SLUGS
    UASInterface* mav = UASManager::instance()->getUASForId(this->uas);
    if (!mav) return;

    if (mav->getAutopilotType() != MAV_AUTOPILOT_SLUGS) {
        // TODO show error popup
        //ui.lastActionLabel->setText(QString("Cannot set slugs nav mode %1 on non-slugs MAV").arg(mode));
        return;
    }

    SlugsMAV* slugsMav = static_cast<SlugsMAV*>(mav);
    slugsMav->setNavMode(mode);
    QString description = slugsMav->getNavModeText(mode);
    ui.lastActionLabel->setText(QString("Sent new mode %1 to %2").arg(description).arg(mav->getUASName()));
#endif
}

/*
void SlugsControlWidget::cycleContextButton()
{
    UAS* mav = dynamic_cast<UAS*>(UASManager::instance()->getUASForId(this->uas));
    if (mav)
    {

        if (!engineOn)
        {
            mav->armSystem();
            ui.lastActionLabel->setText(QString("Enabled motors on %1").arg(mav->getUASName()));
        } else {
            mav->disarmSystem();
            ui.lastActionLabel->setText(QString("Disabled motors on %1").arg(mav->getUASName()));
        }
        // Update state now and in several intervals when MAV might have changed state
        updateStatemachine();

        QTimer::singleShot(50, this, SLOT(updateStatemachine()));
        QTimer::singleShot(200, this, SLOT(updateStatemachine()));

    }

}
*/

