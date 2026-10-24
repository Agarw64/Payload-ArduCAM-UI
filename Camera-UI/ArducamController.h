#pragma once

#include <QObject>
#include <QSerialPort>
#include <QByteArray>
#include <QMap>
#include <QTimer>
#include <QVariantMap>

class ArduCamController : public QObject {
    Q_OBJECT
    Q_PROPERTY(bool connected READ connected NOTIFY connectedChanged)
    Q_PROPERTY(bool streaming READ streaming NOTIFY streamingChanged)
    Q_PROPERTY(int frameCounter READ frameCounter NOTIFY frameCounterChanged)
    Q_PROPERTY(QString lastLogLine READ lastLogLine NOTIFY lastLogLineChanged)

    Q_PROPERTY(bool rawMode READ rawMode NOTIFY rawModeChanged)
    Q_PROPERTY(bool transferActive READ transferActive NOTIFY transferChanged)
    Q_PROPERTY(double transferProgress READ transferProgress NOTIFY transferChanged)
    Q_PROPERTY(int bayerPattern READ bayerPattern WRITE setBayerPattern NOTIFY bayerPatternChanged)
    Q_PROPERTY(int isp5000 READ isp5000 NOTIFY registersChanged)

public:
    explicit ArduCamController(QObject* parent = nullptr);

    bool connected() const { return m_connected; }
    bool streaming() const { return m_streaming; }
    int frameCounter() const { return m_frameCounter; }
    QString lastLogLine() const { return m_lastLogLine; }

    bool rawMode() const { return m_rawMode; }
    bool transferActive() const { return m_transferActive; }
    double transferProgress() const;
    // Preview only: 0 = mosaic (gray), 1 = BGGR, 2 = GBRG, 3 = GRBG, 4 = RGGB
    int bayerPattern() const { return m_bayerPattern; }
    void setBayerPattern(int p);
    // ISP CONTROL 00 as last read back from the sensor, -1 if unknown
    int isp5000() const { return m_registers.value(0x5000, -1); }
    // Every register read back since the last init, as {"0x5000": "0x4F", ...}
    QVariantMap knownRegisters() const;

    Q_INVOKABLE QStringList availablePorts() const;

    Q_INVOKABLE void connectPort(const QString& portName, int baud = 921600);
    Q_INVOKABLE void disconnectPort();

    Q_INVOKABLE void setResolution(int code0to6);
    Q_INVOKABLE void jpegInit();           // 0x11
    Q_INVOKABLE void captureSingle();      // 0x10
    Q_INVOKABLE void startStreaming();     // 0x20
    Q_INVOKABLE void stopStreaming();      // 0x21

    Q_INVOKABLE void rawInit();                 // 0x12
    Q_INVOKABLE void setRawSize(int idx0to3);   // 0x13: 640x480, 1280x960, 1920x1080, 2592x1944
    Q_INVOKABLE void captureRaw();              // 0x14
    Q_INVOKABLE void resendRaw();               // 0x15

    Q_INVOKABLE void writeRegister(int addr, int value);  // 0x16
    Q_INVOKABLE void readRegister(int addr);              // 0x17

    Q_INVOKABLE void setExposureEVIndex(int idx);

    Q_INVOKABLE void setAutoExposure(bool enable);
    Q_INVOKABLE void setExposureUs(quint32 exposureUs);
    Q_INVOKABLE void setLineTimeUs(quint16 lineTimeUs); // optional


    Q_PROPERTY(bool saveSingleShots READ saveSingleShots WRITE setSaveSingleShots NOTIFY saveSingleShotsChanged)

    bool saveSingleShots() const { return m_saveSingleShots; }
    void setSaveSingleShots(bool on);

    bool consumePendingSingleShotSave() {
        if (!m_pendingSingleShotSave) return false;
        m_pendingSingleShotSave = false;
        return true;
    }

    // Call after replacing the preview image so QML reloads it
    void notifyPreviewUpdated();

signals:
    void connectedChanged();
    void streamingChanged();
    void frameCounterChanged();
    void lastLogLineChanged();

    void logLine(const QString& line);
    void jpegFrameReceived(const QByteArray& jpegBytes);
    // Pixel bytes only (width*height, one byte per pixel), CRC already verified
    void rawFrameReceived(const QByteArray& pixels, int width, int height, quint32 crc32);

    void saveSingleShotsChanged();

    void rawModeChanged();
    void transferChanged();
    void bayerPatternChanged();
    void registersChanged();

private slots:
    void onReadyRead();
    void onError(QSerialPort::SerialPortError e);
    void onRawStalled();

private:
    void sendByte(quint8 b);
    void handleTextLine(const QString& line);
    void finishRawFrame();
    void retryRaw(const QString& reason);
    void setTransferActive(bool on);
    void setRawMode(bool on);
    void clearRegisters();

    enum class RxState { Text, Jpeg, Raw };
    RxState m_state = RxState::Text;

    QSerialPort m_serial;
    QByteArray m_rxBuffer;
    QByteArray m_currentJpeg;

    bool m_connected = false;
    bool m_streaming = false;
    int m_frameCounter = 0;
    QString m_lastLogLine;

    bool m_saveSingleShots = false;
    bool m_pendingSingleShotSave = false; // latch: next frame after captureSingle()

    bool m_rawMode = false;
    bool m_transferActive = false;
    int m_bayerPattern = 0;
    QByteArray m_currentRaw;     // pixels + 4 CRC bytes while receiving
    int m_rawWidth = 0;
    int m_rawHeight = 0;
    qint64 m_rawExpected = 0;    // pixel bytes + 4 CRC bytes
    int m_rawRetries = 0;
    QTimer m_rawStallTimer;

    QMap<int, int> m_registers;
};
