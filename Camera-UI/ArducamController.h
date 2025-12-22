#pragma once

#include <QObject>
#include <QSerialPort>
#include <QByteArray>

class ArduCamController : public QObject {
    Q_OBJECT
    Q_PROPERTY(bool connected READ connected NOTIFY connectedChanged)
    Q_PROPERTY(bool streaming READ streaming NOTIFY streamingChanged)
    Q_PROPERTY(int frameCounter READ frameCounter NOTIFY frameCounterChanged)
    Q_PROPERTY(QString lastLogLine READ lastLogLine NOTIFY lastLogLineChanged)

public:
    explicit ArduCamController(QObject* parent = nullptr);

    bool connected() const { return m_connected; }
    bool streaming() const { return m_streaming; }
    int frameCounter() const { return m_frameCounter; }
    QString lastLogLine() const { return m_lastLogLine; }

    Q_INVOKABLE QStringList availablePorts() const;

    Q_INVOKABLE void connectPort(const QString& portName, int baud = 921600);
    Q_INVOKABLE void disconnectPort();

    Q_INVOKABLE void setResolution(int code0to6);
    Q_INVOKABLE void jpegInit();           // 0x11
    Q_INVOKABLE void captureSingle();      // 0x10
    Q_INVOKABLE void startStreaming();     // 0x20
    Q_INVOKABLE void stopStreaming();      // 0x21

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

signals:
    void connectedChanged();
    void streamingChanged();
    void frameCounterChanged();
    void lastLogLineChanged();

    void logLine(const QString& line);
    void jpegFrameReceived(const QByteArray& jpegBytes);

    void saveSingleShotsChanged();

private slots:
    void onReadyRead();
    void onError(QSerialPort::SerialPortError e);

private:
    void sendByte(quint8 b);
    void processTextLines();
    void processJpegBytes();

    enum class RxState { Text, Jpeg };
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
};
