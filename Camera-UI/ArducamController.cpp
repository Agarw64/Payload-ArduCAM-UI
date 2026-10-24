#include "ArduCamController.h"
#include <QSerialPortInfo>

static constexpr int kMaxRawRetries = 3;
static constexpr int kRawStallMs = 3000;

static bool endsWithFF_D9(const QByteArray& data) {
    int n = data.size();
    return (n >= 2 &&
            (quint8)data[n - 2] == 0xFF &&
            (quint8)data[n - 1] == 0xD9);
}

// Same CRC32 as the firmware (and zlib.crc32 / binascii.crc32 in Python)
static quint32 crc32(const char* data, qsizetype len) {
    static quint32 table[256];
    static bool init = false;
    if (!init) {
        for (quint32 i = 0; i < 256; i++) {
            quint32 c = i;
            for (int k = 0; k < 8; k++)
                c = (c & 1) ? (c >> 1) ^ 0xEDB88320u : (c >> 1);
            table[i] = c;
        }
        init = true;
    }
    quint32 crc = 0xFFFFFFFFu;
    for (qsizetype i = 0; i < len; i++)
        crc = table[(crc ^ (quint8)data[i]) & 0xFF] ^ (crc >> 8);
    return ~crc;
}

ArduCamController::ArduCamController(QObject* parent) : QObject(parent) {
    connect(&m_serial, &QSerialPort::readyRead, this, &ArduCamController::onReadyRead);
    connect(&m_serial, &QSerialPort::errorOccurred, this, &ArduCamController::onError);

    m_rawStallTimer.setSingleShot(true);
    m_rawStallTimer.setInterval(kRawStallMs);
    connect(&m_rawStallTimer, &QTimer::timeout, this, &ArduCamController::onRawStalled);
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
    m_currentRaw.clear();
    m_state = RxState::Text;

    // The firmware always boots in JPEG mode
    setRawMode(false);
    setTransferActive(false);
    clearRegisters();
}

void ArduCamController::disconnectPort() {
    if (m_serial.isOpen())
        m_serial.close();

    m_connected = false;
    m_streaming = false;
    m_rawStallTimer.stop();
    m_state = RxState::Text;
    setTransferActive(false);
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
    if (!m_serial.isOpen()) return;
    sendByte(0x11);
    setRawMode(false);
    clearRegisters();
    readRegister(0x5000);   // queued behind the init on the Arduino
}

void ArduCamController::captureSingle()
{
    m_streaming = false;
    emit streamingChanged();

    // Arm saving latch ONLY for single shots
    m_pendingSingleShotSave = m_saveSingleShots;

    sendByte(0x10); // your existing single-shot command
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

void ArduCamController::rawInit() {
    if (!m_serial.isOpen()) return;
    sendByte(0x12);
    setRawMode(true);
    clearRegisters();
    readRegister(0x5000);
}

void ArduCamController::setRawSize(int idx0to3) {
    if (!m_serial.isOpen() || idx0to3 < 0 || idx0to3 > 3) return;
    QByteArray p;
    p.append(char(0x13));
    p.append(char(idx0to3));
    m_serial.write(p);
    m_serial.flush();
    // The size tables rewrite the ISP registers too
    clearRegisters();
    readRegister(0x5000);
}

void ArduCamController::captureRaw() {
    if (!m_serial.isOpen() || m_transferActive) return;
    m_rawRetries = 0;
    setTransferActive(true);
    sendByte(0x14);
}

void ArduCamController::resendRaw() {
    if (!m_serial.isOpen() || m_transferActive) return;
    m_rawRetries = 0;
    setTransferActive(true);
    sendByte(0x15);
}

void ArduCamController::writeRegister(int addr, int value) {
    if (!m_serial.isOpen() || addr < 0 || addr > 0xFFFF || value < 0 || value > 0xFF) return;
    QByteArray p;
    p.append(char(0x16));
    p.append(char(addr & 0xFF));
    p.append(char((addr >> 8) & 0xFF));
    p.append(char(value));
    m_serial.write(p);
    m_serial.flush();
}

void ArduCamController::readRegister(int addr) {
    if (!m_serial.isOpen() || addr < 0 || addr > 0xFFFF) return;
    QByteArray p;
    p.append(char(0x17));
    p.append(char(addr & 0xFF));
    p.append(char((addr >> 8) & 0xFF));
    m_serial.write(p);
    m_serial.flush();
}

double ArduCamController::transferProgress() const {
    if (m_state != RxState::Raw || m_rawExpected <= 0) return 0.0;
    return double(m_currentRaw.size()) / double(m_rawExpected);
}

void ArduCamController::setBayerPattern(int p) {
    if (p < 0 || p > 4 || p == m_bayerPattern) return;
    m_bayerPattern = p;
    emit bayerPatternChanged();
}

QVariantMap ArduCamController::knownRegisters() const {
    QVariantMap out;
    for (auto it = m_registers.cbegin(); it != m_registers.cend(); ++it) {
        out.insert("0x" + QString::number(it.key(), 16).toUpper().rightJustified(4, '0'),
                   "0x" + QString::number(it.value(), 16).toUpper().rightJustified(2, '0'));
    }
    return out;
}

void ArduCamController::notifyPreviewUpdated() {
    m_frameCounter++;
    emit frameCounterChanged();
}

void ArduCamController::setTransferActive(bool on) {
    if (!on) m_rawStallTimer.stop();
    if (m_transferActive == on) return;
    m_transferActive = on;
    emit transferChanged();
}

void ArduCamController::setRawMode(bool on) {
    if (m_rawMode == on) return;
    m_rawMode = on;
    emit rawModeChanged();
}

void ArduCamController::clearRegisters() {
    if (m_registers.isEmpty()) return;
    m_registers.clear();
    emit registersChanged();
}

void ArduCamController::handleTextLine(const QString& line) {
    // Arduino prints this line right before it starts binary JPEG bytes
    if (line.startsWith("ACK IMG END")) {
        m_state = RxState::Jpeg;
        m_currentJpeg.clear();
        return;
    }

    // "ACK RAW <w> <h> <len> END" is followed by exactly <len> pixel bytes + 4 CRC bytes
    if (line.startsWith("ACK RAW ")) {
        const QStringList parts = line.split(' ', Qt::SkipEmptyParts);
        bool okW = false, okH = false, okLen = false;
        const int w = parts.value(2).toInt(&okW);
        const int h = parts.value(3).toInt(&okH);
        const qint64 len = parts.value(4).toLongLong(&okLen);
        if (!okW || !okH || !okLen || w <= 0 || h <= 0 || len != qint64(w) * h) {
            emit logLine("ERROR: Malformed RAW header: " + line);
            return;
        }
        m_rawWidth = w;
        m_rawHeight = h;
        m_rawExpected = len + 4;
        m_currentRaw.clear();
        m_currentRaw.reserve(m_rawExpected);
        m_state = RxState::Raw;
        setTransferActive(true);
        m_rawStallTimer.start();
        emit transferChanged();
        return;
    }

    // "ACK REG <addr hex> <value hex> END"
    if (line.startsWith("ACK REG ")) {
        const QStringList parts = line.split(' ', Qt::SkipEmptyParts);
        bool okA = false, okV = false;
        const int addr = parts.value(2).toInt(&okA, 16);
        const int val = parts.value(3).toInt(&okV, 16);
        if (okA && okV) {
            m_registers.insert(addr, val);
            emit registersChanged();
        }
        return;
    }

    // Firmware gave up on a RAW capture/resend: nothing binary is coming
    if (m_transferActive &&
        (line.contains("RAW capture timeout") || line.contains("RAW frame too short") ||
         line.contains("No RAW frame to resend") || line.contains("needs RAW init"))) {
        setTransferActive(false);
    }
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

                handleTextLine(line);
            }
        } else if (m_state == RxState::Raw) {
            if (!m_rxBuffer.isEmpty()) {
                progressed = true;
                // Take only this frame's bytes; anything after it is text again
                const qint64 need = m_rawExpected - m_currentRaw.size();
                const qint64 take = qMin<qint64>(need, m_rxBuffer.size());
                m_currentRaw += m_rxBuffer.left(take);
                m_rxBuffer.remove(0, take);
                m_rawStallTimer.start();
                emit transferChanged();

                if (m_currentRaw.size() == m_rawExpected)
                    finishRawFrame();
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

void ArduCamController::finishRawFrame() {
    m_rawStallTimer.stop();
    m_state = RxState::Text;

    const qint64 pixelBytes = m_rawExpected - 4;
    const QByteArray pixels = m_currentRaw.left(pixelBytes);
    const uchar* c = reinterpret_cast<const uchar*>(m_currentRaw.constData() + pixelBytes);
    const quint32 sent = quint32(c[0]) | (quint32(c[1]) << 8) | (quint32(c[2]) << 16) | (quint32(c[3]) << 24);
    const quint32 got = crc32(pixels.constData(), pixels.size());
    m_currentRaw.clear();

    if (sent != got) {
        retryRaw(QString("CRC mismatch (sent %1, received %2)")
                     .arg(sent, 8, 16, QChar('0')).arg(got, 8, 16, QChar('0')));
        return;
    }

    emit logLine(QString("RAW frame %1x%2 received, CRC32 %3 OK")
                     .arg(m_rawWidth).arg(m_rawHeight).arg(got, 8, 16, QChar('0')));
    setTransferActive(false);
    emit rawFrameReceived(pixels, m_rawWidth, m_rawHeight, got);

    m_frameCounter++;
    emit frameCounterChanged();
}

void ArduCamController::onRawStalled() {
    if (m_state != RxState::Raw) return;
    retryRaw(QString("transfer stalled at %1 / %2 bytes").arg(m_currentRaw.size()).arg(m_rawExpected));
}

void ArduCamController::retryRaw(const QString& reason) {
    m_state = RxState::Text;
    m_currentRaw.clear();
    m_rxBuffer.clear();

    if (m_rawRetries >= kMaxRawRetries) {
        emit logLine(QString("ERROR: RAW %1. Gave up after %2 resends. Use 'Resend' to try again.")
                         .arg(reason).arg(kMaxRawRetries));
        setTransferActive(false);
        return;
    }
    m_rawRetries++;
    emit logLine(QString("WARNING: RAW %1. Requesting resend %2/%3...")
                     .arg(reason).arg(m_rawRetries).arg(kMaxRawRetries));
    emit transferChanged();
    sendByte(0x15);
}

void ArduCamController::onError(QSerialPort::SerialPortError e) {
    if (e == QSerialPort::NoError) return;
    emit logLine(QString("Serial error: %1").arg(m_serial.errorString()));
}

void ArduCamController::setExposureEVIndex(int idx)
{
    if (!m_serial.isOpen())
        return;

    // Map UI index -> command byte in your sketch
    // 0: -1.7EV  -> 0xA0
    // 1: -1.3EV  -> 0xA1
    // 2: -1.0EV  -> 0xA2
    // 3: -0.7EV  -> 0xA3
    // 4: -0.3EV  -> 0xA4
    // 5: default -> 0xA5
    // 6: +0.7EV  -> 0xA6
    // 7: +1.0EV  -> 0xA7
    // 8: +1.3EV  -> 0xA8
    // 9: +1.7EV  -> 0xA9

    static const quint8 cmds[] = {
        0xA0, 0xA1, 0xA2, 0xA3, 0xA4,
        0xA5,
        0xA6, 0xA7, 0xA8, 0xA9
    };

    if (idx < 0 || idx >= int(sizeof(cmds)/sizeof(cmds[0])))
        return;

    sendByte(cmds[idx]);   // your existing helper that writes a 1-byte command
}

static void appendU32LE(QByteArray& a, quint32 v) {
    a.append(char(v & 0xFF));
    a.append(char((v >> 8) & 0xFF));
    a.append(char((v >> 16) & 0xFF));
    a.append(char((v >> 24) & 0xFF));
}

static void appendU16LE(QByteArray& a, quint16 v) {
    a.append(char(v & 0xFF));
    a.append(char((v >> 8) & 0xFF));
}

void ArduCamController::setAutoExposure(bool enable) {
    if (!m_serial.isOpen()) return;
    QByteArray p;
    p.reserve(2);
    p.append(char(0xF0));
    p.append(char(enable ? 1 : 0));
    m_serial.write(p);
    m_serial.flush();
}

void ArduCamController::setExposureUs(quint32 exposureUs) {
    if (!m_serial.isOpen()) return;
    QByteArray p;
    p.reserve(1 + 4);
    p.append(char(0xF1));
    appendU32LE(p, exposureUs);
    m_serial.write(p);
    m_serial.flush();
}

void ArduCamController::setLineTimeUs(quint16 lineTimeUs) {
    if (!m_serial.isOpen()) return;
    QByteArray p;
    p.reserve(1 + 2);
    p.append(char(0xF2));
    appendU16LE(p, lineTimeUs);
    m_serial.write(p);
    m_serial.flush();
}

void ArduCamController::setSaveSingleShots(bool on)
{
    if (m_saveSingleShots == on) return;
    m_saveSingleShots = on;
    emit saveSingleShotsChanged();
}
