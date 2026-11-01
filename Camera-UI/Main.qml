import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import CameraUI 1.0

ApplicationWindow {
    visible: true
    width: 1100
    height: 650
    title: "ArduCAM Host (Qt)"

    property real zoom: zoomSlider.value

    function hex(v, digits) {
        return ("0000" + v.toString(16).toUpperCase()).slice(-digits)
    }

    // Init and size changes reload the sensor's register tables, which wipes exposure settings
    function reapplyExposure() {
        if (!autoExp.checked) {
            ArduCam.setAutoExposure(false)
            ArduCam.setExposureUs(Math.round(expSlider.value))
        } else if (evCombo.currentIndex !== 5) {
            ArduCam.setExposureEVIndex(evCombo.currentIndex)
        }
    }

    RowLayout {
        anchors.fill: parent
        spacing: 10
        // padding: 10

        // LEFT PANEL (controls)
        Frame {
            Layout.preferredWidth: 360
            Layout.fillHeight: true
            // Size comes from the layout; sizing from the scrolled content would be a binding loop
            contentWidth: 0
            contentHeight: 0

            ScrollView {
                id: leftScroll
                anchors.fill: parent
                contentWidth: availableWidth
                clip: true

            ColumnLayout {
                width: leftScroll.availableWidth
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
                                id: baudCombo
                                Layout.fillWidth: true
                                model: [ "1000000", "500000", "115200" ]
                                currentIndex: 0
                                enabled: !ArduCam.connected
                            }
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            Button {
                                text: ArduCam.connected ? "Close" : "Connect"
                                Layout.fillWidth: true
                                onClicked: {
                                    if (!ArduCam.connected)
                                        ArduCam.connectPort(portCombo.currentText, parseInt(baudCombo.currentText))
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
                            Button {
                                text: "JPEG Init"
                                Layout.fillWidth: true
                                enabled: ArduCam.connected && !ArduCam.streaming && !ArduCam.transferActive
                                onClicked: {
                                    ArduCam.jpegInit()
                                    resCombo.currentIndex = 0   // init leaves the sensor at 320x240
                                    reapplyExposure()
                                }
                            }
                            Button {
                                text: "RAW Init"
                                Layout.fillWidth: true
                                enabled: ArduCam.connected && !ArduCam.streaming && !ArduCam.transferActive
                                onClicked: {
                                    ArduCam.rawInit()
                                    resCombo.currentIndex = 0   // init leaves the sensor at 640x480
                                    reapplyExposure()
                                }
                            }
                        }

                        Label {
                            text: "Mode: " + (ArduCam.rawMode ? "RAW8 Bayer (uncompressed)" : "JPEG")
                            font.bold: true
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            Label { text: "Pix:"; Layout.preferredWidth: 60 }
                            ComboBox {
                                id: resCombo
                                Layout.fillWidth: true
                                enabled: !ArduCam.transferActive
                                model: ArduCam.rawMode
                                       ? [ "640x480", "1280x960", "1920x1080", "2592x1944" ]
                                       : [ "320x240",
                                           "640x480",
                                           "1024x768",
                                           "1280x960",
                                           "1600x1200",
                                           "2048x1536",
                                           "2592x1944" ]
                                onActivated: {
                                    if (ArduCam.rawMode) {
                                        ArduCam.setRawSize(currentIndex)
                                        reapplyExposure()
                                    } else {
                                        ArduCam.setResolution(currentIndex)
                                    }
                                }
                            }
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 10

                            Button {
                                text: ArduCam.rawMode ? "Capture RAW" : "Capture (Single)"
                                Layout.fillWidth: true
                                enabled: ArduCam.connected && !ArduCam.streaming && !ArduCam.transferActive
                                onClicked: {
                                    if (ArduCam.rawMode) ArduCam.captureRaw()
                                    else ArduCam.captureSingle()
                                }
                            }

                            CheckBox {
                                visible: !ArduCam.rawMode
                                text: "Save single shots to ./temp"
                                enabled: ArduCam.connected && !ArduCam.streaming
                                checked: ArduCam.saveSingleShots
                                onToggled: ArduCam.saveSingleShots = checked
                            }

                            Button {
                                visible: ArduCam.rawMode
                                text: "Resend last"
                                enabled: ArduCam.connected && !ArduCam.transferActive
                                onClicked: ArduCam.resendRaw()
                            }
                        }

                        ColumnLayout {
                            Layout.fillWidth: true
                            visible: ArduCam.rawMode
                            spacing: 4

                            Label {
                                text: "RAW shots are always saved to ./temp (.raw + .png + .json)"
                                opacity: 0.6
                                wrapMode: Text.Wrap
                                Layout.fillWidth: true
                            }

                            RowLayout {
                                Layout.fillWidth: true
                                visible: ArduCam.transferActive
                                ProgressBar {
                                    Layout.fillWidth: true
                                    from: 0
                                    to: 1
                                    value: ArduCam.transferProgress
                                }
                                Label { text: Math.round(ArduCam.transferProgress * 100) + "%" }
                            }

                            RowLayout {
                                Layout.fillWidth: true
                                Label { text: "Preview:"; Layout.preferredWidth: 60 }
                                ComboBox {
                                    Layout.fillWidth: true
                                    model: [ "Mosaic (gray)", "BGGR", "GBRG", "GRBG", "RGGB" ]
                                    currentIndex: ArduCam.bayerPattern
                                    onActivated: ArduCam.bayerPattern = currentIndex
                                }
                            }
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            Button {
                                text: ArduCam.streaming ? "Stop Streaming" : "Start Streaming"
                                Layout.fillWidth: true
                                enabled: ArduCam.connected && !ArduCam.rawMode && !ArduCam.transferActive
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
                                    if (!checked)
                                        ArduCam.setExposureUs(Math.round(expSlider.value))
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
                        ColumnLayout {
                            enabled: ArduCam.connected && !autoExp.checked

                            RowLayout {
                                Layout.fillWidth: true
                                Label { text: "Manual exposure (µs)"; }

                                TextField {
                                    id: expField
                                    Layout.preferredWidth: 110
                                    Layout.alignment: Qt.AlignRight
                                    inputMethodHints: Qt.ImhDigitsOnly
                                    // keep the textbox in sync with slider when not actively editing
                                    text: expField.activeFocus
                                          ? expField.text
                                          : Math.round(expSlider.value).toString()

                                    onAccepted: commit()
                                    onEditingFinished: commit()

                                    function commit() {
                                        let v = parseInt(text)
                                        if (isNaN(v)) {
                                            text = Math.round(expSlider.value).toString()
                                            return
                                        }

                                        // clamp to slider range
                                        v = Math.max(expSlider.from, Math.min(expSlider.to, v))

                                        expSlider.value = v
                                        text = v.toString()

                                        // send immediately when user commits textbox
                                        ArduCam.setExposureUs(v)
                                    }
                                }

                                Label { text: "µs" }
                            }

                            Slider {
                                id: expSlider
                                Layout.fillWidth: true
                                from: 100
                                to: 500000
                                value: 5000
                                enabled: ArduCam.connected && !autoExp.checked

                                // update textbox while dragging (nice UX)
                                onMoved: expField.text = Math.round(value).toString()

                                // Send only on release (prevents serial spam)
                                onPressedChanged: {
                                    if (!pressed) {
                                        const v = Math.round(value)
                                        expField.text = v.toString()
                                        ArduCam.setExposureUs(v)
                                    }
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

                GroupBox {
                    title: "ISP Control 00 (0x5000)"
                    Layout.fillWidth: true

                    ColumnLayout {
                        spacing: 4
                        enabled: ArduCam.connected && !ArduCam.transferActive

                        RowLayout {
                            Layout.fillWidth: true
                            Label {
                                Layout.fillWidth: true
                                text: ArduCam.isp5000 < 0 ? "Value: unknown" : "Value: 0x" + hex(ArduCam.isp5000, 2)
                            }
                            Button {
                                text: "Read"
                                onClicked: ArduCam.readRegister(0x5000)
                            }
                        }

                        Repeater {
                            // Bit meanings from OV5642 datasheet table 5-1
                            model: [
                                { bit: 7, name: "LENC correction" },
                                { bit: 6, name: "Gamma (YUV domain)" },
                                { bit: 5, name: "RAW gamma" },
                                { bit: 4, name: "Even odd removing" },
                                { bit: 3, name: "De-noise" },
                                { bit: 2, name: "Black pixel cancellation" },
                                { bit: 1, name: "White pixel cancellation" },
                                { bit: 0, name: "Color interpolation (CIP)" }
                            ]

                            CheckBox {
                                text: "[" + modelData.bit + "] " + modelData.name
                                enabled: ArduCam.isp5000 >= 0
                                checked: ArduCam.isp5000 >= 0 && (ArduCam.isp5000 & (1 << modelData.bit)) !== 0
                                onToggled: {
                                    ArduCam.writeRegister(0x5000, ArduCam.isp5000 ^ (1 << modelData.bit))
                                    // Show what the sensor reports back, not what was clicked
                                    checked = Qt.binding(() => ArduCam.isp5000 >= 0
                                                         && (ArduCam.isp5000 & (1 << modelData.bit)) !== 0)
                                }
                            }
                        }
                    }
                }

                GroupBox {
                    title: "Sensor register (any)"
                    Layout.fillWidth: true

                    ColumnLayout {
                        spacing: 6
                        enabled: ArduCam.connected && !ArduCam.transferActive

                        RowLayout {
                            Layout.fillWidth: true
                            Label { text: "Addr 0x" }
                            TextField {
                                id: regAddr
                                Layout.preferredWidth: 70
                                placeholderText: "5001"
                                validator: RegularExpressionValidator { regularExpression: /[0-9A-Fa-f]{1,4}/ }
                            }
                            Label { text: "Val 0x" }
                            TextField {
                                id: regVal
                                Layout.preferredWidth: 50
                                placeholderText: "FF"
                                validator: RegularExpressionValidator { regularExpression: /[0-9A-Fa-f]{1,2}/ }
                            }
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            Button {
                                text: "Read"
                                Layout.fillWidth: true
                                enabled: regAddr.text.length > 0
                                onClicked: ArduCam.readRegister(parseInt(regAddr.text, 16))
                            }
                            Button {
                                text: "Write"
                                Layout.fillWidth: true
                                enabled: regAddr.text.length > 0 && regVal.text.length > 0
                                onClicked: ArduCam.writeRegister(parseInt(regAddr.text, 16), parseInt(regVal.text, 16))
                            }
                        }

                        Label {
                            text: "Result appears in the log as \"ACK REG <addr> <value>\""
                            opacity: 0.6
                            wrapMode: Text.Wrap
                            Layout.fillWidth: true
                        }
                    }
                }
            }
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
