#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QtQml/qqml.h>

#include "arducamcontroller.h"
#include "FrameProvider.h"

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);
    QQmlApplicationEngine engine;

    ArduCamController controller;
    auto *provider = new FrameProvider();

    engine.addImageProvider("frame", provider);

    // Register as QML singleton: CameraUI 1.0, name "ArduCam"
    qmlRegisterSingletonInstance("CameraUI", 1, 0, "ArduCam", &controller);

    QObject::connect(&controller, &ArduCamController::jpegFrameReceived,
                     &engine, [provider](const QByteArray &jpeg) {
                         QImage img;
                         img.loadFromData(jpeg, "JPG");
                         if (!img.isNull())
                             provider->setFrame(img);
                     });

    engine.loadFromModule("CameraUI", "Main");

    if (engine.rootObjects().isEmpty())
        return -1;

    return app.exec();
}
