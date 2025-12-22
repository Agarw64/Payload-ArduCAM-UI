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

                    TextArea {
                        id: logArea
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        readOnly: true
                        wrapMode: TextArea.NoWrap
                    }
                }
            }
        }
    }

    Connections {
        target: ArduCam
        function onLogLine(line) {
            logArea.append(line)
        }
    }
}
