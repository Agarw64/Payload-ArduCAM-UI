#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QtQml/qqml.h>

#include <QDir>
#include <QDateTime>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>

#include "arducamcontroller.h"
#include "FrameProvider.h"

static const char* const kBayerNames[] = { "mosaic", "BGGR", "GBRG", "GRBG", "RGGB" };

// Preview of an 8-bit Bayer frame. pattern 0 shows the mosaic as grayscale (always
// correct, whatever the real Bayer order is). 1-4 do a half-resolution demosaic:
// each 2x2 cell becomes one RGB pixel.
static QImage renderRaw(const QByteArray& raw, int w, int h, int pattern)
{
    if (raw.size() < qsizetype(w) * h)
        return {};
    const uchar* src = reinterpret_cast<const uchar*>(raw.constData());

    if (pattern <= 0)
        return QImage(src, w, h, w, QImage::Format_Grayscale8).copy();

    // Index of the R and B samples in the cell {(0,0), (1,0), (0,1), (1,1)};
    // the other two are green.
    struct Cell { int r, b; };
    static const Cell cells[4] = { {3, 0}, {2, 1}, {1, 2}, {0, 3} };  // BGGR GBRG GRBG RGGB
    const Cell cell = cells[pattern - 1];

    QImage img(w / 2, h / 2, QImage::Format_RGB888);
    for (int y = 0; y < h / 2; ++y) {
        const uchar* row0 = src + qsizetype(2 * y) * w;
        const uchar* row1 = row0 + w;
        uchar* out = img.scanLine(y);
        for (int x = 0; x < w / 2; ++x) {
            const int c[4] = { row0[2 * x], row0[2 * x + 1], row1[2 * x], row1[2 * x + 1] };
            out[3 * x]     = uchar(c[cell.r]);
            out[3 * x + 1] = uchar((c[0] + c[1] + c[2] + c[3] - c[cell.r] - c[cell.b]) / 2);
            out[3 * x + 2] = uchar(c[cell.b]);
        }
    }
    return img;
}

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

    // Last RAW frame, kept so the preview can be re-rendered with another Bayer order
    struct RawFrame { QByteArray pixels; int w = 0; int h = 0; };
    RawFrame lastRaw;

    QObject::connect(&controller, &ArduCamController::rawFrameReceived,
                     &engine, [&controller, provider, &lastRaw](const QByteArray &pixels, int w, int h, quint32 crc) {
                         lastRaw.pixels = pixels;
                         lastRaw.w = w;
                         lastRaw.h = h;
                         provider->setFrame(renderRaw(pixels, w, h, controller.bayerPattern()));

                         // RAW captures take up to a minute each, so they are always saved
                         QDir dir(QDir::currentPath());
                         if (!dir.exists("temp"))
                             dir.mkdir("temp");

                         const QDateTime now = QDateTime::currentDateTime();
                         const QString base = dir.filePath(QString("temp/raw_%1_%2x%3")
                                                               .arg(now.toString("yyyyMMdd_HHmmss_zzz")).arg(w).arg(h));

                         // .raw: exact sensor bytes, no header
                         QFile f(base + ".raw");
                         if (!f.open(QIODevice::WriteOnly) || f.write(pixels) != pixels.size()) {
                             emit controller.logLine("ERROR: Failed to save: " + f.fileName());
                             return;
                         }
                         f.close();

                         // .png: the same bytes as a lossless 8-bit grayscale image, for quick viewing
                         renderRaw(pixels, w, h, 0).save(base + ".png");

                         // .json: everything needed to decode the .raw
                         QJsonObject meta;
                         meta["file"] = QFileInfo(f).fileName();
                         meta["width"] = w;
                         meta["height"] = h;
                         meta["bits_per_pixel"] = 8;
                         meta["layout"] = "row-major, 1 byte per pixel, no header";
                         meta["bayer_pattern_selected_in_preview"] = kBayerNames[controller.bayerPattern()];
                         meta["crc32"] = QString("%1").arg(crc, 8, 16, QChar('0'));
                         meta["captured_at"] = now.toString(Qt::ISODateWithMs);
                         meta["sensor"] = "OV5642 (ArduCAM Mini 5MP Plus)";
                         meta["registers_read_back"] = QJsonObject::fromVariantMap(controller.knownRegisters());
                         QFile j(base + ".json");
                         if (j.open(QIODevice::WriteOnly))
                             j.write(QJsonDocument(meta).toJson());

                         emit controller.logLine("Saved RAW: " + f.fileName() + " (+ .png, .json)");
                     });

    QObject::connect(&controller, &ArduCamController::bayerPatternChanged,
                     &engine, [&controller, provider, &lastRaw]() {
                         if (lastRaw.pixels.isEmpty())
                             return;
                         provider->setFrame(renderRaw(lastRaw.pixels, lastRaw.w, lastRaw.h, controller.bayerPattern()));
                         controller.notifyPreviewUpdated();
                     });

    engine.loadFromModule("CameraUI", "Main");

    if (engine.rootObjects().isEmpty())
        return -1;

    return app.exec();
}
