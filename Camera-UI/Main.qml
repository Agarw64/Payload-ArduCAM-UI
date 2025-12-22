import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import CameraUI 1.0

ApplicationWindow {
    visible: true
    width: 1000
    height: 600
    title: "ArduCAM Host (Qt)"

    property real zoom: zoomSlider.value

    RowLayout {
        anchors.fill: parent
        spacing: 10
        // padding: 10

        // LEFT PANEL (controls)
        Frame {
            Layout.preferredWidth: 320
            Layout.fillHeight: true

            ColumnLayout {
                anchors.fill: parent
                spacing: 10

                GroupBox {
                    title: "COMPort"
                    Layout.fillWidth: true
                    ColumnLayout {
                        spacing: 8

                        RowLayout {
                            Layout.fillWidth: true
                            Label { text: "Port:"; Layout.preferredWidth: 60 }
                            ComboBox {
                                id: portCombo
                                Layout.fillWidth: true
                                model: ArduCam.availablePorts()
                            }
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            Label { text: "Baud:"; Layout.preferredWidth: 60 }
                            ComboBox {
                                Layout.fillWidth: true
                                model: [ "921600", "115200" ]
                                currentIndex: 0
                            }
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            Button {
                                text: ArduCam.connected ? "Close" : "Connect"
                                Layout.fillWidth: true
                                onClicked: {
                                    if (!ArduCam.connected)
                                        ArduCam.connectPort(portCombo.currentText, 921600)
                                    else
                                        ArduCam.disconnectPort()
                                }
                            }
                        }
                    }
                }

                GroupBox {
                    title: "Camera"
                    Layout.fillWidth: true
                    ColumnLayout {
                        spacing: 8

                        RowLayout {
                            Layout.fillWidth: true
                            Label { text: "Pix:"; Layout.preferredWidth: 60 }
                            ComboBox {
                                id: resCombo
                                Layout.fillWidth: true
                                model: [
                                    "320x240",
                                    "640x480",
                                    "1024x768",
                                    "1280x960",
                                    "1600x1200",
                                    "2048x1536",
                                    "2592x1944"
                                ]
                                onActivated: ArduCam.setResolution(currentIndex)
                            }
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            Button {
                                text: "JPEG Init"
                                Layout.fillWidth: true
                                enabled: ArduCam.connected
                                onClicked: ArduCam.jpegInit()
                            }
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            Button {
                                text: "Capture (Single)"
                                Layout.fillWidth: true
                                enabled: ArduCam.connected && !ArduCam.streaming
                                onClicked: ArduCam.captureSingle()
                            }
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            Button {
                                text: ArduCam.streaming ? "Stop Streaming" : "Start Streaming"
                                Layout.fillWidth: true
                                enabled: ArduCam.connected
                                onClicked: {
                                    if (!ArduCam.streaming) ArduCam.startStreaming()
                                    else ArduCam.stopStreaming()
                                }
                            }
                        }
                    }
                }

                // GroupBox {
                //     title: "Exposure (EV)"
                //     Layout.fillWidth: true

                //     ColumnLayout {
                //         spacing: 8

                //         ComboBox {
                //             id: exposureCombo
                //             Layout.fillWidth: true
                //             enabled: ArduCam.connected

                //             model: [
                //                 "-1.7 EV",
                //                 "-1.3 EV",
                //                 "-1.0 EV",
                //                 "-0.7 EV",
                //                 "-0.3 EV",
                //                 "Default",
                //                 "+0.7 EV",
                //                 "+1.0 EV",
                //                 "+1.3 EV",
                //                 "+1.7 EV"
                //             ]

                //             currentIndex: 5  // default
                //             onActivated: ArduCam.setExposureEVIndex(currentIndex)
                //         }
                //     }
                // }

                GroupBox {
                    title: "Exposure"
                    Layout.fillWidth: true

                    ColumnLayout {
                        spacing: 8

                        // Auto vs manual toggle
                        RowLayout {
                            Layout.fillWidth: true

                            CheckBox {
                                id: autoExp
                                text: "Auto Exposure (EV presets)"
                                checked: true
                                enabled: ArduCam.connected
                                onToggled: {
                                    ArduCam.setAutoExposure(checked)
                                }
                            }
                        }

                        // EV preset dropdown (enabled only in auto mode)
                        ComboBox {
                            id: evCombo
                            Layout.fillWidth: true
                            enabled: ArduCam.connected && autoExp.checked
                            model: ["-1.7 EV","-1.3 EV","-1.0 EV","-0.7 EV","-0.3 EV","Default","+0.7 EV","+1.0 EV","+1.3 EV","+1.7 EV"]
                            currentIndex: 5
                            onActivated: ArduCam.setExposureEVIndex(currentIndex)   // your existing EV function
                        }

                        Rectangle { Layout.fillWidth: true; height: 1; opacity: 0.25 }

                        // Manual exposure time (µs)
                        RowLayout {
                            Layout.fillWidth: true
                            Label { text: "Manual exposure (µs)"; }
                            Label { text: Math.round(expSlider.value) + " µs"; Layout.alignment: Qt.AlignRight }
                        }

                        Slider {
                            id: expSlider
                            Layout.fillWidth: true
                            from: 100      // 0.1 ms
                            to: 50000      // 50 ms (adjust to your needs)
                            value: 5000
                            enabled: ArduCam.connected && !autoExp.checked

                            // Send only on release (prevents serial spam)
                            onPressedChanged: {
                                if (!pressed) {
                                    ArduCam.setExposureUs(Math.round(value))
                                }
                            }
                        }

                        // Optional: line-time calibration
                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 8
                            Label { text: "LineTime (µs)"; }
                            SpinBox {
                                id: lineTimeSpin
                                from: 1
                                to: 200
                                value: 20
                                enabled: ArduCam.connected
                                onValueModified: ArduCam.setLineTimeUs(value)
                            }
                            Label { text: "(calibration)"; opacity: 0.6 }
                        }
                    }
                }

                Item { Layout.fillHeight: true } // spacer
            }
        }

        // RIGHT PANEL (preview + log)
        ColumnLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 10

            Frame {
                Layout.fillWidth: true
                Layout.fillHeight: true

                ColumnLayout {
                    anchors.fill: parent
                    spacing: 8

                    // Preview area
                    Rectangle {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        color: "#111"
                        clip: true

                        Image {
                            id: preview
                            anchors.centerIn: parent
                            source: "image://frame/live?c=" + ArduCam.frameCounter
                            fillMode: Image.PreserveAspectFit
                            smooth: true
                            scale: zoom
                            transformOrigin: Item.Center
                        }
                    }

                    // Zoom + small status row
                    RowLayout {
                        Layout.fillWidth: true
                        Label { text: "Zoom:" }
                        Slider {
                            id: zoomSlider
                            Layout.fillWidth: true
                            from: 0.25
                            to: 3.0
                            value: 1.0
                        }
                        Label { text: Math.round(zoomSlider.value * 100) + "%" }
                    }
                }
            }

            Frame {
                Layout.fillWidth: true
                Layout.preferredHeight: 170

                ColumnLayout {
                    anchors.fill: parent
                    spacing: 6

                    RowLayout {
                        Layout.fillWidth: true
                        Label { text: "Log"; font.bold: true }
                        Item { Layout.fillWidth: true }
                        Button {
                            text: "Clear"
                            onClicked: logArea.text = ""
                        }
                    }

                    ScrollView {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        clip: true

                        TextArea {
                            id: logArea
                            readOnly: true
                            wrapMode: TextArea.NoWrap
                            selectByMouse: true
                            font.family: "Consolas"
                            font.pixelSize: 12

                            // Important so it grows vertically and scrolling works
                            implicitWidth: parent.width
                        }
                    }
                }
            }
        }
    }

    Connections {
        target: ArduCam
        function onLogLine(line) {
            logArea.append(line)

            // Auto-scroll to bottom
            logArea.cursorPosition = logArea.length
        }
    }
}
