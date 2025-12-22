#include "ArduCamController.h"
#include <QSerialPortInfo>

static bool endsWithFF_D9(const QByteArray& data) {
    int n = data.size();
    return (n >= 2 &&
            (quint8)data[n - 2] == 0xFF &&
            (quint8)data[n - 1] == 0xD9);
}

ArduCamController::ArduCamController(QObject* parent) : QObject(parent) {
    connect(&m_serial, &QSerialPort::readyRead, this, &ArduCamController::onReadyRead);
    connect(&m_serial, &QSerialPort::errorOccurred, this, &ArduCamController::onError);
}

QStringList ArduCamController::availablePorts() const {
    QStringList out;
    for (const auto& info : QSerialPortInfo::availablePorts())
        out << info.portName();
    return out;
}

void ArduCamController::connectPort(const QString& portName, int baud) {
    if (m_serial.isOpen())
        m_serial.close();

    m_serial.setPortName(portName);
    m_serial.setBaudRate(baud);
    m_serial.setDataBits(QSerialPort::Data8);
    m_serial.setParity(QSerialPort::NoParity);
    m_serial.setStopBits(QSerialPort::OneStop);
    m_serial.setFlowControl(QSerialPort::NoFlowControl);

    if (!m_serial.open(QIODevice::ReadWrite)) {
        emit logLine(QString("ERROR: Failed to open %1").arg(portName));
        return;
    }

    m_connected = true;
    emit connectedChanged();
    emit logLine(QString("COM %1 is open!").arg(portName));

    // Optional: clear buffers
    m_serial.clear(QSerialPort::AllDirections);
    m_rxBuffer.clear();
    m_currentJpeg.clear();
    m_state = RxState::Text;
}

void ArduCamController::disconnectPort() {
    if (m_serial.isOpen())
        m_serial.close();

    m_connected = false;
    m_streaming = false;
    emit connectedChanged();
    emit streamingChanged();
    emit logLine("COM is closed!");
}

void ArduCamController::sendByte(quint8 b) {
    if (!m_serial.isOpen()) return;
    char c = (char)b;
    m_serial.write(&c, 1);
    m_serial.flush();
}

void ArduCamController::setResolution(int code0to6) {
    if (code0to6 < 0 || code0to6 > 6) return;
    sendByte((quint8)code0to6);
}

void ArduCamController::jpegInit() {
    sendByte(0x11);
}

void ArduCamController::captureSingle() {
    m_streaming = false;
    emit streamingChanged();
    sendByte(0x10);
}

void ArduCamController::startStreaming() {
    m_streaming = true;
    emit streamingChanged();
    sendByte(0x20);
}

void ArduCamController::stopStreaming() {
    sendByte(0x21);
    m_streaming = false;
    emit streamingChanged();
}

void ArduCamController::onReadyRead() {
    m_rxBuffer += m_serial.readAll();

    // Keep draining buffer until no more progress
    bool progressed = true;
    while (progressed) {
        progressed = false;

        if (m_state == RxState::Text) {
            int nl = m_rxBuffer.indexOf('\n');
            if (nl >= 0) {
                progressed = true;
                QByteArray lineBytes = m_rxBuffer.left(nl + 1);
                m_rxBuffer.remove(0, nl + 1);

                QString line = QString::fromLatin1(lineBytes).trimmed();
                if (!line.isEmpty()) {
                    m_lastLogLine = line;
                    emit lastLogLineChanged();
                    emit logLine(line);
                }

                // Arduino prints this line right before it starts binary JPEG bytes
                if (line.startsWith("ACK IMG END")) {
                    m_state = RxState::Jpeg;
                    m_currentJpeg.clear();
                }
            }
        } else {
            // RxState::Jpeg
            if (!m_rxBuffer.isEmpty()) {
                progressed = true;
                m_currentJpeg += m_rxBuffer;
                m_rxBuffer.clear();

                // We detect JPEG end marker
                if (endsWithFF_D9(m_currentJpeg)) {
                    emit jpegFrameReceived(m_currentJpeg);

                    m_frameCounter++;
                    emit frameCounterChanged();

                    m_currentJpeg.clear();
                    m_state = RxState::Text;
                } else {
                    // Optional: safety cap so a broken stream doesn't blow RAM
                    const int maxJpeg = 1024 * 1024; // 1MB
                    if (m_currentJpeg.size() > maxJpeg) {
                        emit logLine("ERROR: JPEG buffer overflow (no FF D9 found). Resetting parser.");
                        m_currentJpeg.clear();
                        m_state = RxState::Text;
                    }
                }
            }
        }
    }
}

void ArduCamController::onError(QSerialPort::SerialPortError e) {
    if (e == QSerialPort::NoError) return;
    emit logLine(QString("Serial error: %1").arg(m_serial.errorString()));
}
