# Payload ArduCAM UI

Qt host app (`Camera-UI/`) and Arduino firmware (`Camera-Firmware/`) for the ArduCAM Mini 5MP Plus (OV5642).

The ArduCAM library's `memorysaver.h` must have `#define OV5642_MINI_5MP_PLUS` enabled (and every other platform commented out).

## Running the app

Extract **`release-raw.zip`** and run `release-raw\appCamera-UI.exe` from the extracted folder. It includes all the Qt DLLs; no Qt install is needed.

`release-debug.zip` is the older JPEG-only build. Don't run the `.exe` in the `release-debug/` folder directly: its DLLs were never committed (`*.dll` is gitignored), so it fails with "Qt6Gui.dll was not found".

To rebuild: Qt 6.5.3 MinGW 64-bit with Qt Serial Port. Build `Camera-UI/CMakeLists.txt` in Release, then run `windeployqt --qmldir Camera-UI appCamera-UI.exe` (without `--release`, which fails with "Unable to find the platform plugin" on MinGW builds).

## RAW capture

1. Connect at 1000000 baud, then click **RAW Init**. The sensor starts at 640x480.
2. Pick a size (640x480, 1280x960, 1920x1080, 2592x1944) and click **Capture RAW**.
   The firmware and app must come from the same version: RAW frames are sent in acknowledged blocks, and a mismatched pair stalls.
3. Each frame is CRC-checked. A corrupted or stalled transfer is re-sent automatically (up to 3 times) without taking a new photo.
4. Every RAW capture is saved to `./temp`:
   - `raw_<time>_<w>x<h>.raw`: the sensor bytes exactly as received. Row-major, 8 bits per pixel, Bayer mosaic, no header.
   - `.png`: the same bytes as a lossless 8-bit grayscale image (the Bayer mosaic is visible).
   - `.json`: width, height, CRC32, and the ISP registers read back before the capture.

The Bayer order at 640x480 is BGGR, confirmed on hardware with a solid red target (the bottom-right site of each 2x2 cell read 74 against about 32 for the other three). Other sizes have not been checked, and mirror/flip changes the order.

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

A RAW frame on the wire is `ACK RAW <w> <h> <len> END\r\n`, then `<len>` pixel bytes, then the CRC32 of those bytes (4 bytes, little endian; same as Python's `zlib.crc32`). The host sends one `0xFE` byte after the header line and after every 97 pixel bytes except the last block, and the firmware waits for it before sending more. The Uno's USB-serial chip has a 128-byte buffer that overwrites itself during USB pauses, so only one block is ever in flight. Native-USB boards (Leonardo) have USB flow control instead, so after the first `0xFE` the firmware discards the rest, and they need RTS or DTR set on the port before they send anything. If no `0xFE` arrives within 2 s the firmware abandons the frame and the host's 3 s stall timer requests a resend. Register replies are `ACK REG <addr hex> <value hex> END`.
