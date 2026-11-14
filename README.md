# Payload ArduCAM UI

Qt host app (`Camera-UI/`) and Arduino firmware (`Camera-Firmware/`) for the ArduCAM Mini 5MP Plus (OV5642).

The ArduCAM library's `memorysaver.h` must have `#define OV5642_MINI_5MP_PLUS` enabled (and every other platform commented out).

## Running the app

**Windows:** get the latest repo (`git pull`, or **Code → Download ZIP** on GitHub) and run **`release-raw\appCamera-UI.exe`**. It includes all the Qt DLLs; no Qt install is needed. The Arduino must run the firmware from the same version (see RAW capture).

`release-raw/` is built automatically: every push that changes `Camera-UI/` runs [`.github/workflows/windows-release.yml`](.github/workflows/windows-release.yml) on a GitHub Windows machine (Qt 6.5.3 MinGW 64-bit + `windeployqt`), which commits the new build to `release-raw/` on the same branch a few minutes later. Pull after that commit appears. To rebuild without a code change, use **Actions → Build Windows app → Run workflow**. Nobody needs Windows or Qt installed to update it.

`release-debug.zip` is the older JPEG-only build. Don't run the `.exe` in the `release-debug/` folder directly: its DLLs were never committed, so it fails with "Qt6Gui.dll was not found".

To build by hand on Windows: Qt 6.5.3 MinGW 64-bit with Qt Serial Port. Build `Camera-UI/CMakeLists.txt` in Release, then run `windeployqt --qmldir Camera-UI appCamera-UI.exe` (without `--release`, which fails with "Unable to find the platform plugin" on MinGW builds).

### macOS

The `.exe` is Windows-only; build a native Mac app from the same source instead:

```sh
brew install qt cmake
cmake -S Camera-UI -B Camera-UI/build-mac -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$(brew --prefix qt)"
cmake --build Camera-UI/build-mac -j 8
open Camera-UI/build-mac/appCamera-UI.app
```

The serial port shows up as `cu.usbmodem...`. Opened from Finder, the app saves to `~/Documents/ArduCAM-UI/temp` (a Finder launch has no writable working folder).

## RAW capture

1. Connect at 1000000 baud, then click **RAW Init**. The sensor starts at 640x480.
2. Pick a size (640x480, 1280x960, 1920x1080, 2592x1944) and click **Capture RAW**.
   The firmware and app must come from the same version: RAW frames are sent in acknowledged blocks, and a mismatched pair stalls.
   **Frames** sets how many frames one click takes (e.g. 20 darks at the same exposure). They are taken one after another, each saved as its own file, at roughly one frame per transfer time (about 3+ s at 640x480). **Stop** finishes the current frame and stops. If a frame fails all its resends, the batch stops and the log says how many were saved.
3. Each frame is CRC-checked. A corrupted or stalled transfer is re-sent automatically (up to 3 times) without taking a new photo.
4. Every RAW capture is saved to `./temp`:
   - `raw_<time>_<w>x<h>.raw`: the sensor bytes exactly as received. Row-major, 8 bits per pixel, Bayer mosaic, no header.
   - `.png`: the same bytes as a lossless 8-bit grayscale image (the Bayer mosaic is visible).
   - `.json`: width, height, CRC32, the ISP registers read back before the capture, and `sensor_capture` (see below).
   - The `.png` also carries the metadata in its text chunks (`ArduCAM.Metadata` = the whole JSON, plus `ArduCAM.exposure_lines`, `ArduCAM.gain_register_value`, ...). Read them with `PIL.Image.open(f).text`. The `.raw` stays header-less by design; use the `.json` next to it.
   - JPEG single shots saved to `./temp` get a `.json` too, and the same JSON is written into the JPEG as a COM segment.

## Capture metadata

Right after each capture the firmware reads a fixed list of sensor registers (exposure `0x3500-0x3502`, AEC/AGC mode `0x3503`, gain `0x3508-0x350B`, max exposure `0x350C/D`, HTS/VTS `0x380C-0x380F`, PLL `0x3010-0x3012`, mirror/flip `0x3818`, AEC targets and gain ceiling, ISP `0x5000`) and prints them as `ACK REG` lines followed by `ACK META row_time_ns=<n> row_time_ok=<0|1> manual_exposure=<0|1> END`. This is before the multi-second transfer, so in auto mode the values are the sensor's state for that frame rather than where the AEC drifted to afterwards (in auto mode the frame can still be 1-2 frames off, since the sensor latches exposure at frame boundaries). The host decodes them using the OV5642 datasheet (v1.11) formulas into `sensor_capture`:

- `exposure_lines` = `{0x3500[3:0], 0x3501, 0x3502} / 16` (datasheet 4.6.2). This is the trustworthy exposure value.
- `exposure_us` = `exposure_lines * row_time_ns / 1000`. The row time is the measured per-size value the firmware uses (87.5 µs for 640x480 and 1280x960, 133.33 µs for 1920x1080 and 2592x1944; `results/HANDOFF-row-time-lookup-table.md`). It is only valid while the sensor timing registers are unchanged: the firmware reports `row_time_ok` (checks `0x3010` and HTS) and the JSON carries it as `row_time_table_valid`. JPEG modes have no measured row time, so `exposure_us` is omitted there and the firmware refuses microsecond exposure commands outside RAW mode.
- `gain_register_value` is the raw 9-bit value from `0x350A/0x350B`. The datasheet gives no multiplier formula, so none is computed.
- `hts`, `vts`, `max_exposure_lines`, `aec_mode`/`agc_mode`/`vts_mode`, `mirror`, `vertical_flip`, and every register as `registers`.

If the firmware is older and sends no snapshot, `capture_time_registers` is `false` and `registers` are just the last values read.

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
