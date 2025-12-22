#pragma once
#include <QQuickImageProvider>
#include <QImage>
#include <QMutex>

class FrameProvider : public QQuickImageProvider {
public:
    FrameProvider() : QQuickImageProvider(QQuickImageProvider::Image) {}

    void setFrame(const QImage& img) {
        QMutexLocker lock(&m_mutex);
        m_frame = img;
    }

    QImage requestImage(const QString&, QSize* size, const QSize& requestedSize) override {
        QMutexLocker lock(&m_mutex);
        if (size) *size = m_frame.size();
        if (requestedSize.isValid())
            return m_frame.scaled(requestedSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);
        return m_frame;
    }

private:
    QImage m_frame;
    QMutex m_mutex;
};
