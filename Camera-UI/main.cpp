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

// Copies the headline numbers of a capture into PNG text chunks (readable with PIL's
// Image.text / exiftool / pngcheck), plus the whole metadata as one JSON blob.
// Everything is ASCII so Qt writes plain tEXt chunks.
static void embedPngText(QImage& img, const QJsonObject& meta)
{
    img.setText("ArduCAM.Metadata", QString::fromUtf8(QJsonDocument(meta).toJson(QJsonDocument::Compact)));
    img.setText("Software", "Payload-ArduCAM-UI");

    const QJsonObject s = meta.value("sensor_capture").toObject();
    static const char* const keys[] = {
        "aec_mode", "agc_mode", "exposure_lines", "exposure_register_value",
        "row_time_ns", "exposure_us", "gain_register_value", "hts", "vts", "mirror", "vertical_flip"
    };
    for (const char* k : keys) {
        const QJsonValue v = s.value(k);
        if (v.isUndefined())
            continue;
        const QString text = v.isBool() ? (v.toBool() ? "true" : "false")
                           : v.isString() ? v.toString()
                           : QString::number(v.toDouble(), 'g', 10);
        img.setText(QString("ArduCAM.") + k, text);
    }
    for (const char* k : { "captured_at", "width", "height", "crc32" }) {
        const QJsonValue v = meta.value(k);
        if (!v.isUndefined())
            img.setText(QString("ArduCAM.") + k, v.isString() ? v.toString() : QString::number(v.toInt()));
    }
}

// Inserts the metadata as a JPEG COM segment, after any leading APPn segments (JFIF/Exif
// must stay first). Returns the input unchanged if it doesn't look like a JPEG.
static QByteArray jpegWithComment(const QByteArray& jpeg, const QByteArray& text)
{
    const auto u = [&jpeg](qsizetype i) { return quint8(jpeg.at(i)); };
    if (jpeg.size() < 4 || u(0) != 0xFF || u(1) != 0xD8)
        return jpeg;
    qsizetype pos = 2;
    while (pos + 4 <= jpeg.size() && u(pos) == 0xFF && u(pos + 1) >= 0xE0 && u(pos + 1) <= 0xEF) {
        const qsizetype next = pos + 2 + ((u(pos + 2) << 8) | u(pos + 3));
        if (next > jpeg.size())
            return jpeg;
        pos = next;
    }
    const QByteArray body = text.left(65533);
    QByteArray seg;
    seg.append(char(0xFF)).append(char(0xFE));
    seg.append(char((body.size() + 2) >> 8)).append(char((body.size() + 2) & 0xFF));
    seg.append(body);
    return jpeg.left(pos) + seg + jpeg.mid(pos);
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

                             QJsonObject meta;
                             meta["file"] = QFileInfo(path).fileName();
                             meta["captured_at"] = QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
                             meta["sensor"] = "OV5642 (ArduCAM Mini 5MP Plus)";
                             meta["format"] = "JPEG";
                             meta["sensor_capture"] = controller.captureMetadata();
                             const QByteArray metaJson = QJsonDocument(meta).toJson();

                             QFile f(path);
                             if (f.open(QIODevice::WriteOnly)) {
                                 f.write(jpegWithComment(jpeg, QJsonDocument(meta).toJson(QJsonDocument::Compact)));
                                 f.close();
                                 QFile j(dir.filePath("temp/shot_" + ts + ".json"));
                                 if (j.open(QIODevice::WriteOnly))
                                     j.write(metaJson);
                                 emit controller.logLine("Saved single shot: " + path + " (+ .json, metadata in JPEG comment)");
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

                         // .json: everything needed to decode the .raw (also embedded in the .png)
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
                         // Sensor state when the frame was taken (exposure, gain, timing...), not after the transfer
                         meta["sensor_capture"] = controller.captureMetadata();
                         QFile j(base + ".json");
                         if (j.open(QIODevice::WriteOnly))
                             j.write(QJsonDocument(meta).toJson());

                         // .png: the same bytes as a lossless 8-bit grayscale image, for quick viewing,
                         // with the metadata in its text chunks
                         QImage png = renderRaw(pixels, w, h, 0);
                         embedPngText(png, meta);
                         png.save(base + ".png");

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
