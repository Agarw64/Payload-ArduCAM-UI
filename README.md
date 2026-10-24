# Payload ArduCAM UI

Qt host app (`Camera-UI/`) and Arduino firmware (`Camera-Firmware/`) for the ArduCAM Mini 5MP Plus (OV5642).

The ArduCAM library's `memorysaver.h` must have `#define OV5642_MINI_5MP_PLUS` enabled (and every other platform commented out).

## RAW capture

1. Connect, then click **RAW Init**. The sensor starts at 640x480.
2. Pick a size (640x480, 1280x960, 1920x1080, 2592x1944) and click **Capture RAW**.
   Approximate transfer time on an Uno: 3 s, 13 s, 22 s and 55 s respectively.
3. Each frame is CRC-checked. A corrupted or stalled transfer is re-sent automatically (up to 3 times) without taking a new photo.
4. Every RAW capture is saved to `./temp`:
   - `raw_<time>_<w>x<h>.raw`: the sensor bytes exactly as received. Row-major, 8 bits per pixel, Bayer mosaic, no header.
   - `.png`: the same bytes as a lossless 8-bit grayscale image (the Bayer mosaic is visible).
   - `.json`: width, height, CRC32, and the ISP registers read back before the capture.

The Bayer order (BGGR/GBRG/GRBG/RGGB) has not been confirmed on hardware yet. To find it, capture the colour-bar test pattern and switch the **Preview** dropdown until the colours look right. Mirror/flip changes the order.

Loading in Python:

```python
import numpy as np
img = np.fromfile("raw_..._1280x960.raw", dtype=np.uint8).reshape(960, 1280)
```

## ISP / registers

- **ISP Control 00 (0x5000)** shows the value read back from the sensor, with one checkbox per bit (OV5642 datasheet table 5-1). Every Init or RAW size change reloads the register tables, which resets these bits; the panel re-reads them automatically.
- **Sensor register** reads or writes any 16-bit-address sensor register, for everything else in datasheet section 5.

## Serial commands added

| Byte | Payload | Meaning |
|---|---|---|
| `0x12` | | RAW init (640x480) |
| `0x13` | u8 size index 0-3 | RAW size |
| `0x14` | | Capture RAW and send it |
| `0x15` | | Re-send the last RAW frame from the FIFO |
| `0x16` | u16 addr LE, u8 value | Write a sensor register (replies with a read-back) |
| `0x17` | u16 addr LE | Read a sensor register |

A RAW frame on the wire is `ACK RAW <w> <h> <len> END\r\n`, then `<len>` pixel bytes, then the CRC32 of those bytes (4 bytes, little endian; same as Python's `zlib.crc32`). Register replies are `ACK REG <addr hex> <value hex> END`.
