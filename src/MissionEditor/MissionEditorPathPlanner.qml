/*=====================================================================

QGroundControl Open Source Ground Control Station

(c) 2009, 2015 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>

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

import QtQuick          2.4
import QtQuick.Controls 1.3
import QtQuick.Dialogs  1.2

import QGroundControl               1.0
import QGroundControl.ScreenTools   1.0
import QGroundControl.Controls      1.0
import QGroundControl.Palette       1.0

Rectangle {
    visible:        mapPathPlannerButton.checked
    color:          qgcPal.window
    opacity:        _rightPanelOpacity
    z:              QGroundControl.zOrderTopMost

    Column {
        anchors.margins:    _margin
        anchors.fill:       parent
        visible:            true

        QGCLabel {
            font.pixelSize: ScreenTools.mediumFontPixelSize
            text:           "Path Planner"
        }

        Item {
            width: 10
            height: ScreenTools.defaultFontPixelHeight
        }

        QGCLabel {
            id:         mapPathPlannerPanelLabel
            width:      parent.width
            wrapMode:   Text.WordWrap
            // FIXME update text when the flag is changed
            text: !_pathPlannerWantsCoverage ? "This is used to plan an efficient path by reordering all mission items for the active vehicle." : "This will plan an efficient path to cover the designated polygon."
        }

        Item {
            width: 10
            height: ScreenTools.defaultFontPixelHeight
        }

        Item {
            width:  parent.width
            height: turnRadiusField.height

            QGCLabel {
                anchors.baseline:   turnRadiusField.baseline
                text:               "Turn radius:"
            }

            QGCTextField {
                id:             turnRadiusField
                showUnits:		true
                unitsLabel:		"[m]"
                anchors.right:  parent.right
                width:          _editFieldWidth
                text:           _pathPlannerTurnRadius

                /*
                Keys.onPressed: {
                        if (event.key == Qt.Key_Enter || event.key == Qt.Key_Return) {
                            console.log("Turn radius changed to " + Number(turnRadiusField.text))
                            _pathPlannerTurnRadius = Number(turnRadiusField.text)
                        }
                }
                */
                onTextChanged: {
                    console.log("Turn radius changed to " + Number(turnRadiusField.text))
                    _pathPlannerTurnRadius = Number(turnRadiusField.text)
                }
            }
        }

        Item {
            width: 10
            height: ScreenTools.defaultFontPixelHeight / 3
        }

        Item {
            width:  parent.width
            height: sensorWidthField.height
            enabled: _pathPlannerWantsCoverage

            QGCLabel {
                anchors.baseline:   sensorWidthField.baseline
                text:               "Sensor width:"
            }

            QGCTextField {
                id:             sensorWidthField
                showUnits:		true
                unitsLabel:		"[m]"
                anchors.right:  parent.right
                width:          _editFieldWidth
                text:           _pathPlannerSensorWidth

                /*
                Keys.onPressed: {
                        if (event.key == Qt.Key_Enter || event.key == Qt.Key_Return) {
                            console.log("Sensor width changed to " + Number(sensorWidthField.text))
                            _pathPlannerSensorWidth = Number(sensorWidthField.text)
                        }
                }
                */
                onTextChanged: {
                    console.log("Sensor width changsed to " + Number(sensorWidthField.text))
                    _pathPlannerSensorWidth = Number(sensorWidthField.text)
                }
            }
        }
        // FIXME add algorithm option

        Item {
            width: 10
            height: ScreenTools.defaultFontPixelHeight / 3
        }


        Item {
            width: parent.width
            height: homeAsInitialPositionLabel.height
            enabled: _activeVehicle

            QGCLabel {
                id:                 homeAsInitialPositionLabel
                //anchors.baseline:   homeAsInitialPositionField.baseline
                width: 				parent.width
                text:   "Use the home as the initial position when planning. If this " +
                        "is unchecked, the active vehicle's current position will be " +
                        "used instead."
                wrapMode: Text.WordWrap
            }

            CheckBox {
                id:             homeAsInitialPositionField
                anchors.left:   homeAsInitialPositionLabel.right
                anchors.baseline: homeAsInitialPositionLabel.bottom
                ///anchors.bottom: homeAsInitialPositionLabel.bottom
                anchors.leftMargin: -55 // FIXME anchor to end of last line of text
                checked:        _pathPlannerHomeAsInitial
            }
        }

        Item {
            width: 10
            height: ScreenTools.defaultFontPixelHeight / 3
        }
        /*
          // FIXME add return to initial option
        Item {
            width:  parent.width
            height: returnToInitialField.height

            QGCLabel {
                id:                 returnToInitialLabel
                anchors.baseline:   returnToInitialField.baseline
                text:               "Return to current position: "
            }

            CheckBox {
                id:             returnToInitialField
                anchors.left:   returnToInitialLabel.right
                anchors.leftMargin: 5
                checked:        _pathPlannerReturnToInitial
            }
        }
        */

        Item {
            width: 10
            height: ScreenTools.defaultFontPixelHeight / 3
        }

        Row {
            spacing: ScreenTools.defaultFontPixelWidth

            QGCButton {
                text: "Plan Path"

                onClicked: {
                    if (!_pathPlannerWantsCoverage) {
                        controller.planMissionItemSequence(_pathPlannerTurnRadius)
                        _missionItems = controller.missionItems
                    }
                    else if (controller.hasValidCoveragePolygon()) {
                        controller.planCoverageArea( _pathPlannerTurnRadius, _pathPlannerSensorWidth)
                        _missionItems = controller.missionItems
                        // Remove the polygon
                        // FIXME not sure if this behavior is correct
                        //_pathPlannerWantsCoverage = false
                        //clearCoveragePolygon()
                    }
                }
            }

        }
    } // Column - Offline view
    // FIXME add online view
} // Item - Path Planner Manager

