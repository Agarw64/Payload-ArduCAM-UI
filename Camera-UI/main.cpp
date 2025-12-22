#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QtQml/qqml.h>

#include <QDir>
#include <QDateTime>
#include <QFile>

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
                     &engine, [&controller, provider](const QByteArray &jpeg) {
                         // update preview
                         QImage img;
                         img.loadFromData(jpeg, "JPG");
                         if (!img.isNull())
                             provider->setFrame(img);

                         // Save ONLY if (a) checkbox enabled AND (b) it was armed by captureSingle()
                         if (controller.saveSingleShots() && controller.consumePendingSingleShotSave()) {

                             QDir dir(QDir::currentPath());
                             if (!dir.exists("temp"))
                                 dir.mkdir("temp");

                             const QString ts = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz");
                             const QString path = dir.filePath("temp/shot_" + ts + ".jpg");

                             QFile f(path);
                             if (f.open(QIODevice::WriteOnly)) {
                                 f.write(jpeg);
                                 f.close();
                                 emit controller.logLine("Saved single shot: " + path);
                             } else {
                                 emit controller.logLine("ERROR: Failed to save: " + path);
                             }
                         }
                     });

    engine.loadFromModule("CameraUI", "Main");

    if (engine.rootObjects().isEmpty())
        return -1;

    return app.exec();
}
