// ArduCAM Mini demo (C)2017 Lee
// Web: http://www.ArduCAM.com
// This program is a demo of how to use most of the functions
// of the library with ArduCAM Mini camera, and can run on any Arduino platform.
// This demo was made for ArduCAM_Mini_5MP_Plus.
// It needs to be used in combination with PC software.
// It can test OV5642 functions.
//

// This program requires the ArduCAM V4.0.0 (or later) library and ArduCAM_Mini_5MP_Plus
// and use Arduino IDE 1.6.8 compiler or above
#include <Wire.h>
#include <ArduCAM.h>
#include <SPI.h>
// #include "memorysaver.h"
//This demo can only work on OV5642_MINI_5MP_Plus  platform.
//#if !(defined OV5642_MINI_5MP_PLUS)
//  #error Please select the hardware platform and camera module in the ../libraries/ArduCAM/memorysaver.h file
//#endif
#define BMPIMAGEOFFSET 66
const char bmp_header[BMPIMAGEOFFSET] PROGMEM =
{
  0x42, 0x4D, 0x36, 0x58, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 0x00, 0x28, 0x00,
  0x00, 0x00, 0x40, 0x01, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x01, 0x00, 0x10, 0x00, 0x03, 0x00,
  0x00, 0x00, 0x00, 0x58, 0x02, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0xE0, 0x07, 0x00, 0x00, 0x1F, 0x00,
  0x00, 0x00
};

// set pin 7 as the slave select for the digital pot:
const int CS = 7;
bool is_header = false;
int mode = 0;
uint8_t start_capture = 0;
 ArduCAM myCAM( OV5642, CS );
uint8_t read_fifo_burst(ArduCAM myCAM);


#define CMD_SET_AUTO_EXPOSURE   0xF0
#define CMD_SET_EXPOSURE_US     0xF1
#define CMD_SET_LINETIME_US     0xF2

// RAW (Bayer, 8 bits per pixel) capture.
// Frame on the wire: "ACK RAW <w> <h> <len> END\r\n", then exactly <len> pixel
// bytes, then the CRC32 of those bytes (4 bytes, little endian; same as zlib.crc32).
// The host sends RAW_BLOCK_ACK after the header line and after every RAW_BLOCK_BYTES
// pixel bytes except the last block; nothing more is sent until it arrives. The Uno's
// 16U2 USB chip has a 128-byte buffer that silently overwrites itself if USB stalls,
// so never having more than one block in flight is what keeps frames intact.
#define CMD_RAW_INIT            0x12
#define CMD_SET_RAW_SIZE        0x13  // + 1 byte: 0=640x480 1=1280x960 2=1920x1080 3=2592x1944
#define CMD_CAPTURE_RAW         0x14
#define CMD_RESEND_RAW          0x15  // re-send the frame still in the FIFO (no new capture)

// Sensor register access (works in any mode, including while streaming).
// Both reply with "ACK REG <addr hex> <value hex> END".
#define CMD_WRITE_REG           0x16  // + u16 addr LE + u8 value
#define CMD_READ_REG            0x17  // + u16 addr LE

// Set to 1 if the FIFO turns out to return a dummy byte at the start of a burst read
// (symptom: FIFO length reported as w*h+1 and the image is shifted by one pixel).
#define RAW_BURST_DUMMY_BYTE    0
#define RAW_CAPTURE_TIMEOUT_MS  5000UL
#if defined(USBCON)
// Native USB has its own flow control, so the host's block ACKs are just discarded
#define RAW_BLOCK_BYTES         64
#else
// The 16U2 forwards once it holds more than 96 bytes, otherwise it waits for a 4 ms timer
#define RAW_BLOCK_BYTES         97
#endif
#define RAW_BLOCK_ACK           0xFE  // not a command byte, so a late ACK is ignored
#define RAW_ACK_TIMEOUT_MS      2000UL

static const uint8_t  RAW_SIZES[4] = { OV5642_640x480, OV5642_1280x960, OV5642_1920x1080, OV5642_2592x1944 };
static const uint16_t RAW_W[4]     = { 640, 1280, 1920, 2592 };
static const uint16_t RAW_H[4]     = { 480,  960, 1080, 1944 };

static bool g_rawMode = false;
static uint8_t g_rawSizeIdx = 0;
static bool g_rawFrameReady = false;   // FIFO holds a complete RAW frame that can be resent
static uint16_t g_rawFrameW = 0;
static uint16_t g_rawFrameH = 0;

static const uint32_t CRC32_TABLE[256] PROGMEM = {
  0x00000000UL, 0x77073096UL, 0xEE0E612CUL, 0x990951BAUL, 0x076DC419UL, 0x706AF48FUL,
  0xE963A535UL, 0x9E6495A3UL, 0x0EDB8832UL, 0x79DCB8A4UL, 0xE0D5E91EUL, 0x97D2D988UL,
  0x09B64C2BUL, 0x7EB17CBDUL, 0xE7B82D07UL, 0x90BF1D91UL, 0x1DB71064UL, 0x6AB020F2UL,
  0xF3B97148UL, 0x84BE41DEUL, 0x1ADAD47DUL, 0x6DDDE4EBUL, 0xF4D4B551UL, 0x83D385C7UL,
  0x136C9856UL, 0x646BA8C0UL, 0xFD62F97AUL, 0x8A65C9ECUL, 0x14015C4FUL, 0x63066CD9UL,
  0xFA0F3D63UL, 0x8D080DF5UL, 0x3B6E20C8UL, 0x4C69105EUL, 0xD56041E4UL, 0xA2677172UL,
  0x3C03E4D1UL, 0x4B04D447UL, 0xD20D85FDUL, 0xA50AB56BUL, 0x35B5A8FAUL, 0x42B2986CUL,
  0xDBBBC9D6UL, 0xACBCF940UL, 0x32D86CE3UL, 0x45DF5C75UL, 0xDCD60DCFUL, 0xABD13D59UL,
  0x26D930ACUL, 0x51DE003AUL, 0xC8D75180UL, 0xBFD06116UL, 0x21B4F4B5UL, 0x56B3C423UL,
  0xCFBA9599UL, 0xB8BDA50FUL, 0x2802B89EUL, 0x5F058808UL, 0xC60CD9B2UL, 0xB10BE924UL,
  0x2F6F7C87UL, 0x58684C11UL, 0xC1611DABUL, 0xB6662D3DUL, 0x76DC4190UL, 0x01DB7106UL,
  0x98D220BCUL, 0xEFD5102AUL, 0x71B18589UL, 0x06B6B51FUL, 0x9FBFE4A5UL, 0xE8B8D433UL,
  0x7807C9A2UL, 0x0F00F934UL, 0x9609A88EUL, 0xE10E9818UL, 0x7F6A0DBBUL, 0x086D3D2DUL,
  0x91646C97UL, 0xE6635C01UL, 0x6B6B51F4UL, 0x1C6C6162UL, 0x856530D8UL, 0xF262004EUL,
  0x6C0695EDUL, 0x1B01A57BUL, 0x8208F4C1UL, 0xF50FC457UL, 0x65B0D9C6UL, 0x12B7E950UL,
  0x8BBEB8EAUL, 0xFCB9887CUL, 0x62DD1DDFUL, 0x15DA2D49UL, 0x8CD37CF3UL, 0xFBD44C65UL,
  0x4DB26158UL, 0x3AB551CEUL, 0xA3BC0074UL, 0xD4BB30E2UL, 0x4ADFA541UL, 0x3DD895D7UL,
  0xA4D1C46DUL, 0xD3D6F4FBUL, 0x4369E96AUL, 0x346ED9FCUL, 0xAD678846UL, 0xDA60B8D0UL,
  0x44042D73UL, 0x33031DE5UL, 0xAA0A4C5FUL, 0xDD0D7CC9UL, 0x5005713CUL, 0x270241AAUL,
  0xBE0B1010UL, 0xC90C2086UL, 0x5768B525UL, 0x206F85B3UL, 0xB966D409UL, 0xCE61E49FUL,
  0x5EDEF90EUL, 0x29D9C998UL, 0xB0D09822UL, 0xC7D7A8B4UL, 0x59B33D17UL, 0x2EB40D81UL,
  0xB7BD5C3BUL, 0xC0BA6CADUL, 0xEDB88320UL, 0x9ABFB3B6UL, 0x03B6E20CUL, 0x74B1D29AUL,
  0xEAD54739UL, 0x9DD277AFUL, 0x04DB2615UL, 0x73DC1683UL, 0xE3630B12UL, 0x94643B84UL,
  0x0D6D6A3EUL, 0x7A6A5AA8UL, 0xE40ECF0BUL, 0x9309FF9DUL, 0x0A00AE27UL, 0x7D079EB1UL,
  0xF00F9344UL, 0x8708A3D2UL, 0x1E01F268UL, 0x6906C2FEUL, 0xF762575DUL, 0x806567CBUL,
  0x196C3671UL, 0x6E6B06E7UL, 0xFED41B76UL, 0x89D32BE0UL, 0x10DA7A5AUL, 0x67DD4ACCUL,
  0xF9B9DF6FUL, 0x8EBEEFF9UL, 0x17B7BE43UL, 0x60B08ED5UL, 0xD6D6A3E8UL, 0xA1D1937EUL,
  0x38D8C2C4UL, 0x4FDFF252UL, 0xD1BB67F1UL, 0xA6BC5767UL, 0x3FB506DDUL, 0x48B2364BUL,
  0xD80D2BDAUL, 0xAF0A1B4CUL, 0x36034AF6UL, 0x41047A60UL, 0xDF60EFC3UL, 0xA867DF55UL,
  0x316E8EEFUL, 0x4669BE79UL, 0xCB61B38CUL, 0xBC66831AUL, 0x256FD2A0UL, 0x5268E236UL,
  0xCC0C7795UL, 0xBB0B4703UL, 0x220216B9UL, 0x5505262FUL, 0xC5BA3BBEUL, 0xB2BD0B28UL,
  0x2BB45A92UL, 0x5CB36A04UL, 0xC2D7FFA7UL, 0xB5D0CF31UL, 0x2CD99E8BUL, 0x5BDEAE1DUL,
  0x9B64C2B0UL, 0xEC63F226UL, 0x756AA39CUL, 0x026D930AUL, 0x9C0906A9UL, 0xEB0E363FUL,
  0x72076785UL, 0x05005713UL, 0x95BF4A82UL, 0xE2B87A14UL, 0x7BB12BAEUL, 0x0CB61B38UL,
  0x92D28E9BUL, 0xE5D5BE0DUL, 0x7CDCEFB7UL, 0x0BDBDF21UL, 0x86D3D2D4UL, 0xF1D4E242UL,
  0x68DDB3F8UL, 0x1FDA836EUL, 0x81BE16CDUL, 0xF6B9265BUL, 0x6FB077E1UL, 0x18B74777UL,
  0x88085AE6UL, 0xFF0F6A70UL, 0x66063BCAUL, 0x11010B5CUL, 0x8F659EFFUL, 0xF862AE69UL,
  0x616BFFD3UL, 0x166CCF45UL, 0xA00AE278UL, 0xD70DD2EEUL, 0x4E048354UL, 0x3903B3C2UL,
  0xA7672661UL, 0xD06016F7UL, 0x4969474DUL, 0x3E6E77DBUL, 0xAED16A4AUL, 0xD9D65ADCUL,
  0x40DF0B66UL, 0x37D83BF0UL, 0xA9BCAE53UL, 0xDEBB9EC5UL, 0x47B2CF7FUL, 0x30B5FFE9UL,
  0xBDBDF21CUL, 0xCABAC28AUL, 0x53B39330UL, 0x24B4A3A6UL, 0xBAD03605UL, 0xCDD70693UL,
  0x54DE5729UL, 0x23D967BFUL, 0xB3667A2EUL, 0xC4614AB8UL, 0x5D681B02UL, 0x2A6F2B94UL,
  0xB40BBE37UL, 0xC30C8EA1UL, 0x5A05DF1BUL, 0x2D02EF8DUL
};

static uint16_t g_lineTimeUs = 20;  // starting guess; you can calibrate via CMD_SET_LINETIME_US
static bool g_manualExposure = false;

static bool readExact(uint8_t* dst, size_t n, unsigned long timeoutMs = 200) {
  unsigned long start = millis();
  while (Serial.available() < (int)n) {
    if (millis() - start > timeoutMs) return false;
  }
  Serial.readBytes(dst, n);
  return true;
}

static uint16_t readU16LE(bool* ok) {
  uint8_t b[2];
  if (!readExact(b, 2)) { *ok = false; return 0; }
  *ok = true;
  return (uint16_t)b[0] | ((uint16_t)b[1] << 8);
}

static uint32_t readU32LE(bool* ok) {
  uint8_t b[4];
  if (!readExact(b, 4)) { *ok = false; return 0; }
  *ok = true;
  return (uint32_t)b[0] |
         ((uint32_t)b[1] << 8) |
         ((uint32_t)b[2] << 16) |
         ((uint32_t)b[3] << 24);
}

static void setAutoExposureOV5642(bool enable) {
  // Common OV5642 pattern: 0x3503 controls manual/auto AEC/AGC.
  // 0x00 = auto, 0x07 = manual AEC + manual AGC (common setting).
  // If your image looks weird, we can tweak this value.
  myCAM.wrSensorReg16_8(0x3503, enable ? 0x00 : 0x07);
  g_manualExposure = !enable;
}

static void setExposureUsOV5642(uint32_t exposureUs) {
  // Force manual mode so exposure “sticks”
  setAutoExposureOV5642(false);

  // Convert µs -> “exposure units”. Many OV sensors use 20-bit exposure in units of 1/16 line.
  // exp20 = lines * 16
  if (g_lineTimeUs == 0) g_lineTimeUs = 1;
  uint32_t exp20 = (exposureUs * 16UL) / (uint32_t)g_lineTimeUs;

  // Clamp to 20-bit (0xFFFFF)
  if (exp20 > 0xFFFFF) exp20 = 0xFFFFF;

  // Write to exposure registers (common packing)
  // 0x3500: [19:16], 0x3501: [15:8], 0x3502: [7:0]
  myCAM.wrSensorReg16_8(0x3500, (exp20 >> 16) & 0x0F);
  myCAM.wrSensorReg16_8(0x3501, (exp20 >> 8) & 0xFF);
  myCAM.wrSensorReg16_8(0x3502, (exp20) & 0xFF);
}

static void printRegister(uint16_t addr) {
  uint8_t val = 0;
  myCAM.rdSensorReg16_8(addr, &val);
  Serial.print(F("ACK REG "));
  Serial.print(addr, HEX);
  Serial.print(' ');
  Serial.print(val, HEX);
  Serial.println(F(" END"));
}

static void rawInit() {
  // Same sequence as ArduCAM's ArduCAM_Mini_5MP_Plus_OV5642_RAW example.
  mode = 0;
  start_capture = 0;
  myCAM.set_format(RAW);
  myCAM.InitCAM();                 // leaves the sensor at 640x480 RAW
  myCAM.set_bit(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
  delay(1000);
  g_rawMode = true;
  g_rawSizeIdx = 0;
  g_rawFrameReady = false;
  Serial.println(F("ACK CMD RAW init 640x480 END"));
}

static bool waitForBlockAck() {
  unsigned long start = millis();
  while (millis() - start <= RAW_ACK_TIMEOUT_MS) {
    if (Serial.available() && Serial.read() == RAW_BLOCK_ACK)
      return true;
  }
  return false;
}

static void sendRawFrame(bool rewind) {
  uint32_t len = (uint32_t)g_rawFrameW * g_rawFrameH;
  Serial.print(F("ACK RAW "));
  Serial.print(g_rawFrameW);
  Serial.print(' ');
  Serial.print(g_rawFrameH);
  Serial.print(' ');
  Serial.print(len);
  Serial.println(F(" END"));

  if (!waitForBlockAck())
    return;   // the host's stall timer asks for a resend

  if (rewind)
    myCAM.write_reg(ARDUCHIP_FIFO, FIFO_RDPTR_RST_MASK);

  uint8_t buf[RAW_BLOCK_BYTES];
  uint32_t crc = 0xFFFFFFFFUL;
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();
#if RAW_BURST_DUMMY_BYTE
  SPI.transfer(0x00);
#endif
  while (len) {
    uint8_t n = len > sizeof(buf) ? sizeof(buf) : (uint8_t)len;
    for (uint8_t i = 0; i < n; i++) {
      uint8_t b = SPI.transfer(0x00);
      buf[i] = b;
      crc = pgm_read_dword(&CRC32_TABLE[(uint8_t)(crc ^ b)]) ^ (crc >> 8);
    }
    // Chunked writes: much faster than per-byte writes on native-USB boards (Leonardo)
    Serial.write(buf, n);
    len -= n;
#if defined(USBCON)
    while (Serial.available())
      Serial.read();
#else
    if (len && !waitForBlockAck()) {
      myCAM.CS_HIGH();
      return;
    }
#endif
  }
  myCAM.CS_HIGH();

  crc = ~crc;
  for (uint8_t i = 0; i < 4; i++)
    Serial.write((uint8_t)(crc >> (8 * i)));
}

static void captureRaw() {
  g_rawFrameReady = false;
  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
  myCAM.start_capture();

  unsigned long start = millis();
  while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) {
    if (millis() - start > RAW_CAPTURE_TIMEOUT_MS) {
      Serial.println(F("ACK CMD RAW capture timeout END"));
      return;
    }
  }

  g_rawFrameW = RAW_W[g_rawSizeIdx];
  g_rawFrameH = RAW_H[g_rawSizeIdx];
  uint32_t expected = (uint32_t)g_rawFrameW * g_rawFrameH;
  uint32_t fifoLen = myCAM.read_fifo_length();
  Serial.print(F("ACK CMD RAW capture done, FIFO "));
  Serial.print(fifoLen);
  Serial.print(F(" bytes, expected "));
  Serial.print(expected);
  Serial.println(F(" END"));
  if (fifoLen < expected) {
    Serial.println(F("ACK CMD RAW frame too short END"));
    return;
  }

  // The capture-done flag is left set on purpose: the frame stays in the FIFO so
  // CMD_RESEND_RAW can send it again. The next capture flushes it.
  g_rawFrameReady = true;
  sendRawFrame(false);
}

// Sensor settings shared by the idle loop and the streaming loop.
// Returns false if cmd is not a sensor setting.
static bool handleSensorCommand(uint8_t cmd) {
  switch (cmd)
  {
    case 0x40:
      myCAM.OV5642_set_Light_Mode(Advanced_AWB);
      Serial.println(F("ACK CMD Set to Advanced_AWB END"));break;
    case 0x41:
      myCAM.OV5642_set_Light_Mode(Simple_AWB);
      Serial.println(F("ACK CMD Set to Simple_AWB END"));break;
    case 0x42:
      myCAM.OV5642_set_Light_Mode(Manual_day);
      Serial.println(F("ACK CMD Set to Manual_day END"));break;
    case 0x43:
      myCAM.OV5642_set_Light_Mode(Manual_A);
      Serial.println(F("ACK CMD Set to Manual_A END"));break;
    case 0x44:
      myCAM.OV5642_set_Light_Mode(Manual_cwf);
      Serial.println(F("ACK CMD Set to Manual_cwf END"));break;
    case 0x45:
      myCAM.OV5642_set_Light_Mode(Manual_cloudy);
      Serial.println(F("ACK CMD Set to Manual_cloudy END"));break;
    case 0x50:
      myCAM.OV5642_set_Color_Saturation(Saturation4);
      Serial.println(F("ACK CMD Set to Saturation+4 END"));break;
    case 0x51:
      myCAM.OV5642_set_Color_Saturation(Saturation3);
      Serial.println(F("ACK CMD Set to Saturation+3 END"));break;
    case 0x52:
      myCAM.OV5642_set_Color_Saturation(Saturation2);
      Serial.println(F("ACK CMD Set to Saturation+2 END"));break;
    case 0x53:
      myCAM.OV5642_set_Color_Saturation(Saturation1);
      Serial.println(F("ACK CMD Set to Saturation+1 END"));break;
    case 0x54:
      myCAM.OV5642_set_Color_Saturation(Saturation0);
      Serial.println(F("ACK CMD Set to Saturation+0 END"));break;
    case 0x55:
      myCAM.OV5642_set_Color_Saturation(Saturation_1);
      Serial.println(F("ACK CMD Set to Saturation-1 END"));break;
    case 0x56:
      myCAM.OV5642_set_Color_Saturation(Saturation_2);
      Serial.println(F("ACK CMD Set to Saturation-2 END"));break;
    case 0x57:
      myCAM.OV5642_set_Color_Saturation(Saturation_3);
      Serial.println(F("ACK CMD Set to Saturation-3 END"));break;
    case 0x58:
      myCAM.OV5642_set_Color_Saturation(Saturation_4);
      Serial.println(F("ACK CMD Set to Saturation-4 END"));break;
    case 0x60:
      myCAM.OV5642_set_Brightness(Brightness4);
      Serial.println(F("ACK CMD Set to Brightness+4 END"));break;
    case 0x61:
      myCAM.OV5642_set_Brightness(Brightness3);
      Serial.println(F("ACK CMD Set to Brightness+3 END"));break;
    case 0x62:
      myCAM.OV5642_set_Brightness(Brightness2);
      Serial.println(F("ACK CMD Set to Brightness+2 END"));break;
    case 0x63:
      myCAM.OV5642_set_Brightness(Brightness1);
      Serial.println(F("ACK CMD Set to Brightness+1 END"));break;
    case 0x64:
      myCAM.OV5642_set_Brightness(Brightness0);
      Serial.println(F("ACK CMD Set to Brightness+0 END"));break;
    case 0x65:
      myCAM.OV5642_set_Brightness(Brightness_1);
      Serial.println(F("ACK CMD Set to Brightness-1 END"));break;
    case 0x66:
      myCAM.OV5642_set_Brightness(Brightness_2);
      Serial.println(F("ACK CMD Set to Brightness-2 END"));break;
    case 0x67:
      myCAM.OV5642_set_Brightness(Brightness_3);
      Serial.println(F("ACK CMD Set to Brightness-3 END"));break;
    case 0x68:
      myCAM.OV5642_set_Brightness(Brightness_4);
      Serial.println(F("ACK CMD Set to Brightness-4 END"));break;
    case 0x70:
      myCAM.OV5642_set_Contrast(Contrast4);
      Serial.println(F("ACK CMD Set to Contrast+4 END"));break;
    case 0x71:
      myCAM.OV5642_set_Contrast(Contrast3);
      Serial.println(F("ACK CMD Set to Contrast+3 END"));break;
    case 0x72:
      myCAM.OV5642_set_Contrast(Contrast2);
      Serial.println(F("ACK CMD Set to Contrast+2 END"));break;
    case 0x73:
      myCAM.OV5642_set_Contrast(Contrast1);
      Serial.println(F("ACK CMD Set to Contrast+1 END"));break;
    case 0x74:
      myCAM.OV5642_set_Contrast(Contrast0);
      Serial.println(F("ACK CMD Set to Contrast+0 END"));break;
    case 0x75:
      myCAM.OV5642_set_Contrast(Contrast_1);
      Serial.println(F("ACK CMD Set to Contrast-1 END"));break;
    case 0x76:
      myCAM.OV5642_set_Contrast(Contrast_2);
      Serial.println(F("ACK CMD Set to Contrast-2 END"));break;
    case 0x77:
      myCAM.OV5642_set_Contrast(Contrast_3);
      Serial.println(F("ACK CMD Set to Contrast-3 END"));break;
    case 0x78:
      myCAM.OV5642_set_Contrast(Contrast_4);
      Serial.println(F("ACK CMD Set to Contrast-4 END"));break;
    case 0x80:
      myCAM.OV5642_set_hue(degree_180);
      Serial.println(F("ACK CMD Set to -180 degree END"));break;
    case 0x81:
      myCAM.OV5642_set_hue(degree_150);
      Serial.println(F("ACK CMD Set to -150 degree END"));break;
    case 0x82:
      myCAM.OV5642_set_hue(degree_120);
      Serial.println(F("ACK CMD Set to -120 degree END"));break;
    case 0x83:
      myCAM.OV5642_set_hue(degree_90);
      Serial.println(F("ACK CMD Set to -90 degree END"));break;
    case 0x84:
      myCAM.OV5642_set_hue(degree_60);
      Serial.println(F("ACK CMD Set to -60 degree END"));break;
    case 0x85:
      myCAM.OV5642_set_hue(degree_30);
      Serial.println(F("ACK CMD Set to -30 degree END"));break;
    case 0x86:
      myCAM.OV5642_set_hue(degree_0);
      Serial.println(F("ACK CMD Set to 0 degree END"));break;
    case 0x87:
      myCAM.OV5642_set_hue(degree30);
      Serial.println(F("ACK CMD Set to 30 degree END"));break;
    case 0x88:
      myCAM.OV5642_set_hue(degree60);
      Serial.println(F("ACK CMD Set to 60 degree END"));break;
    case 0x89:
      myCAM.OV5642_set_hue(degree90);
      Serial.println(F("ACK CMD Set to 90 degree END"));break;
    case 0x8a:
      myCAM.OV5642_set_hue(degree120);
      Serial.println(F("ACK CMD Set to 120 degree END"));break;
    case 0x8b:
      myCAM.OV5642_set_hue(degree150);
      Serial.println(F("ACK CMD Set to 150 degree END"));break;
    case 0x90:
      myCAM.OV5642_set_Special_effects(Normal);
      Serial.println(F("ACK CMD Set to Normal END"));break;
    case 0x91:
      myCAM.OV5642_set_Special_effects(BW);
      Serial.println(F("ACK CMD Set to BW END"));break;
    case 0x92:
      myCAM.OV5642_set_Special_effects(Bluish);
      Serial.println(F("ACK CMD Set to Bluish END"));break;
    case 0x93:
      myCAM.OV5642_set_Special_effects(Sepia);
      Serial.println(F("ACK CMD Set to Sepia END"));break;
    case 0x94:
      myCAM.OV5642_set_Special_effects(Reddish);
      Serial.println(F("ACK CMD Set to Reddish END"));break;
    case 0x95:
      myCAM.OV5642_set_Special_effects(Greenish);
      Serial.println(F("ACK CMD Set to Greenish END"));break;
    case 0x96:
      myCAM.OV5642_set_Special_effects(Negative);
      Serial.println(F("ACK CMD Set to Negative END"));break;
    case 0xA0:
      myCAM.OV5642_set_Exposure_level(Exposure_17_EV);
      Serial.println(F("ACK CMD Set to -1.7EV END"));break;
    case 0xA1:
      myCAM.OV5642_set_Exposure_level(Exposure_13_EV);
      Serial.println(F("ACK CMD Set to -1.3EV END"));break;
    case 0xA2:
      myCAM.OV5642_set_Exposure_level(Exposure_10_EV);
      Serial.println(F("ACK CMD Set to -1.0EV END"));break;
    case 0xA3:
      myCAM.OV5642_set_Exposure_level(Exposure_07_EV);
      Serial.println(F("ACK CMD Set to -0.7EV END"));break;
    case 0xA4:
      myCAM.OV5642_set_Exposure_level(Exposure_03_EV);
      Serial.println(F("ACK CMD Set to -0.3EV END"));break;
    case 0xA5:
      myCAM.OV5642_set_Exposure_level(Exposure_default);
      Serial.println(F("ACK CMD Set to -Exposure_default END"));break;
    case 0xA6:
      myCAM.OV5642_set_Exposure_level(Exposure07_EV);
      Serial.println(F("ACK CMD Set to 0.7EV END"));break;
    case 0xA7:
      myCAM.OV5642_set_Exposure_level(Exposure10_EV);
      Serial.println(F("ACK CMD Set to 1.0EV END"));break;
    case 0xA8:
      myCAM.OV5642_set_Exposure_level(Exposure13_EV);
      Serial.println(F("ACK CMD Set to 1.3EV END"));break;
    case 0xA9:
      myCAM.OV5642_set_Exposure_level(Exposure17_EV);
      Serial.println(F("ACK CMD Set to 1.7EV END"));break;
    case 0xB0:
      myCAM.OV5642_set_Sharpness(Auto_Sharpness_default);
      Serial.println(F("ACK CMD Set to Auto Sharpness default END"));break;
    case 0xB1:
      myCAM.OV5642_set_Sharpness(Auto_Sharpness1);
      Serial.println(F("ACK CMD Set to Auto Sharpness +1 END"));break;
    case 0xB2:
      myCAM.OV5642_set_Sharpness(Auto_Sharpness2);
      Serial.println(F("ACK CMD Set to Auto Sharpness +2 END"));break;
    case 0xB3:
      myCAM.OV5642_set_Sharpness(Manual_Sharpnessoff);
      Serial.println(F("ACK CMD Set to Auto Manual Sharpness off END"));break;
    case 0xB4:
      myCAM.OV5642_set_Sharpness(Manual_Sharpness1);
      Serial.println(F("ACK CMD Set to Auto Manual Sharpness +1 END"));break;
    case 0xB5:
      myCAM.OV5642_set_Sharpness(Manual_Sharpness2);
      Serial.println(F("ACK CMD Set to Auto Manual Sharpness +2 END"));break;
    case 0xB6:
      myCAM.OV5642_set_Sharpness(Manual_Sharpness3);
      Serial.println(F("ACK CMD Set to Auto Manual Sharpness +3 END"));break;
    case 0xB7:
      myCAM.OV5642_set_Sharpness(Manual_Sharpness4);
      Serial.println(F("ACK CMD Set to Auto Manual Sharpness +4 END"));break;
    case 0xB8:
      myCAM.OV5642_set_Sharpness(Manual_Sharpness5);
      Serial.println(F("ACK CMD Set to Auto Manual Sharpness +5 END"));break;
    case 0xC0:
      myCAM.OV5642_set_Mirror_Flip(MIRROR);
      Serial.println(F("ACK CMD Set to MIRROR END"));break;
    case 0xC1:
      myCAM.OV5642_set_Mirror_Flip(FLIP);
      Serial.println(F("ACK CMD Set to FLIP END"));break;
    case 0xC2:
      myCAM.OV5642_set_Mirror_Flip(MIRROR_FLIP);
      Serial.println(F("ACK CMD Set to MIRROR&FLIP END"));break;
    case 0xC3:
      myCAM.OV5642_set_Mirror_Flip(Normal);
      Serial.println(F("ACK CMD Set to Normal END"));break;
    case 0xD0:
      myCAM.OV5642_set_Compress_quality(high_quality);
      Serial.println(F("ACK CMD Set to high quality END"));break;
    case 0xD1:
      myCAM.OV5642_set_Compress_quality(default_quality);
      Serial.println(F("ACK CMD Set to default quality END"));break;
    case 0xD2:
      myCAM.OV5642_set_Compress_quality(low_quality);
      Serial.println(F("ACK CMD Set to low quality END"));break;
    case 0xE0:
      myCAM.OV5642_Test_Pattern(Color_bar);
      Serial.println(F("ACK CMD Set to Color bar END"));break;
    case 0xE1:
      myCAM.OV5642_Test_Pattern(Color_square);
      Serial.println(F("ACK CMD Set to Color square END"));break;
    case 0xE2:
      myCAM.OV5642_Test_Pattern(BW_square);
      Serial.println(F("ACK CMD Set to BW square END"));break;
    case 0xE3:
      myCAM.OV5642_Test_Pattern(DLI);
      Serial.println(F("ACK CMD Set to DLI END"));break;
    case CMD_SET_AUTO_EXPOSURE: {
      uint8_t v = 0;
      if (!readExact(&v, 1)) { Serial.println(F("ACK CMD AE timeout END")); break; }

      setAutoExposureOV5642(v != 0);

      Serial.print(F("ACK CMD AE "));
      Serial.print(v ? F("AUTO") : F("MANUAL"));
      Serial.println(F(" END"));
      break;
    }
    case CMD_SET_EXPOSURE_US: {
      bool ok = false;
      uint32_t us = readU32LE(&ok);
      if (!ok) { Serial.println(F("ACK CMD EXP timeout END")); break; }

      setExposureUsOV5642(us);

      Serial.print(F("ACK CMD EXP_US "));
      Serial.print(us);
      Serial.println(F(" END"));
      break;
    }
    case CMD_SET_LINETIME_US: {
      bool ok = false;
      uint16_t lt = readU16LE(&ok);
      if (!ok) { Serial.println(F("ACK CMD LT timeout END")); break; }

      if (lt < 1) lt = 1;
      g_lineTimeUs = lt;

      Serial.print(F("ACK CMD LINETIME_US "));
      Serial.print(g_lineTimeUs);
      Serial.println(F(" END"));
      break;
    }
    case CMD_WRITE_REG: {
      bool ok = false;
      uint16_t addr = readU16LE(&ok);
      uint8_t val = 0;
      if (ok) ok = readExact(&val, 1);
      if (!ok) { Serial.println(F("ACK CMD REG timeout END")); break; }

      myCAM.wrSensorReg16_8(addr, val);
      printRegister(addr);   // read back so the host sees what actually stuck
      break;
    }
    case CMD_READ_REG: {
      bool ok = false;
      uint16_t addr = readU16LE(&ok);
      if (!ok) { Serial.println(F("ACK CMD REG timeout END")); break; }

      printRegister(addr);
      break;
    }
    default:
      return false;
  }
  return true;
}


void setup() {
// put your setup code here, to run once:
uint8_t vid, pid;
uint8_t temp;
#if defined(__SAM3X8E__)
  Wire1.begin();
  Serial.begin(115200);
#else
  Wire.begin();
  // 1000000 is exact on a 16 MHz AVR; 921600 really runs at 1000000 and corrupts RAW frames
  Serial.begin(1000000);
#endif
Serial.println(F("ACK CMD ArduCAM Start! END"));
// set the CS as an output:
pinMode(CS, OUTPUT);
digitalWrite(CS, HIGH);
// initialize SPI:
SPI.begin();
 //Reset the CPLD
myCAM.write_reg(0x07, 0x80);
delay(100);
myCAM.write_reg(0x07, 0x00);
delay(100);
while(1){
  //Check if the ArduCAM SPI bus is OK
  myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
  temp = myCAM.read_reg(ARDUCHIP_TEST1);
  if (temp != 0x55){
    Serial.println(F("ACK CMD SPI interface Error! END"));
    delay(1000);continue;
  }else{
    Serial.println(F("ACK CMD SPI interface OK. END"));break;
  }
}
  while(1){
    //Check if the camera module type is OV5642
    myCAM.wrSensorReg16_8(0xff, 0x01);
    myCAM.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
    myCAM.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
    if((vid != 0x56) || (pid != 0x42)){
      Serial.println(F("ACK CMD Can't find OV5642 module! END"));
      delay(1000);continue;
    }
    else{
      Serial.println(F("ACK CMD OV5642 detected. END"));break;
    }
  }
//Change to JPEG capture mode and initialize the OV5642 module
myCAM.set_format(JPEG);
myCAM.InitCAM();

  myCAM.write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);   //VSYNC is active HIGH
  myCAM.OV5642_set_JPEG_size(OV5642_320x240);
delay(1000);
myCAM.clear_fifo_flag();
myCAM.write_reg(ARDUCHIP_FRAMES,0x00);
}
void loop() {
// put your main code here, to run repeatedly:
uint8_t temp = 0xff, temp_last = 0;
bool is_header = false;
if (Serial.available())
{
  temp = Serial.read();

  // JPEG/BMP size and capture commands would misread a RAW-configured sensor
  if (g_rawMode && (temp <= 0x06 || temp == 0x10 || temp == 0x20 || temp == 0x30))
  {
    Serial.println(F("ACK CMD Not available in RAW mode, send JPEG Init first END"));
    temp = 0xff;
  }

  switch (temp)
  {
    case 0:
      myCAM.OV5642_set_JPEG_size(OV5642_320x240);delay(1000);
      Serial.println(F("ACK CMD switch to OV5642_320x240 END"));
    temp = 0xff;
    break;
    case 1:
      myCAM.OV5642_set_JPEG_size(OV5642_640x480);delay(1000);
      Serial.println(F("ACK CMD switch to OV5642_640x480 END"));
    temp = 0xff;
    break;
    case 2:
      myCAM.OV5642_set_JPEG_size(OV5642_1024x768);delay(1000);
      Serial.println(F("ACK CMD switch to OV5642_1024x768 END"));
    temp = 0xff;
    break;
    case 3:
    temp = 0xff;
      myCAM.OV5642_set_JPEG_size(OV5642_1280x960);delay(1000);
      Serial.println(F("ACK CMD switch to OV5642_1280x960 END"));
    break;
    case 4:
    temp = 0xff;
      myCAM.OV5642_set_JPEG_size(OV5642_1600x1200);delay(1000);
      Serial.println(F("ACK CMD switch to OV5642_1600x1200 END"));
    break;
    case 5:
    temp = 0xff;
      myCAM.OV5642_set_JPEG_size(OV5642_2048x1536);delay(1000);
      Serial.println(F("ACK CMD switch to OV5642_2048x1536 END"));
    break;
    case 6:
    temp = 0xff;
      myCAM.OV5642_set_JPEG_size(OV5642_2592x1944);delay(1000);
      Serial.println(F("ACK CMD switch to OV5642_2592x1944 END"));
    break;
    case 0x10:
    mode = 1;
    temp = 0xff;
    start_capture = 1;
    Serial.println(F("ACK CMD CAM start single shoot. END"));
    break;
    case 0x11:
    temp = 0xff;
    g_rawMode = false;
    g_rawFrameReady = false;
    myCAM.set_format(JPEG);
    myCAM.InitCAM();
    #if !(defined (OV2640_MINI_2MP))
    myCAM.set_bit(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
    #endif
    Serial.println(F("ACK CMD JPEG init 320x240 END"));
    break;
    case CMD_RAW_INIT:
    temp = 0xff;
    rawInit();
    break;
    case CMD_SET_RAW_SIZE: {
      temp = 0xff;
      uint8_t idx = 0;
      if (!readExact(&idx, 1)) { Serial.println(F("ACK CMD RAW size timeout END")); break; }
      if (!g_rawMode) { Serial.println(F("ACK CMD RAW size needs RAW init first END")); break; }
      if (idx >= sizeof(RAW_SIZES)) { Serial.println(F("ACK CMD RAW size invalid END")); break; }

      myCAM.OV5642_set_RAW_size(RAW_SIZES[idx]);delay(1000);
      g_rawSizeIdx = idx;
      Serial.print(F("ACK CMD RAW size "));
      Serial.print(RAW_W[idx]);
      Serial.print('x');
      Serial.print(RAW_H[idx]);
      Serial.println(F(" END"));
      break;
    }
    case CMD_CAPTURE_RAW:
    temp = 0xff;
    if (!g_rawMode) { Serial.println(F("ACK CMD RAW capture needs RAW init first END")); break; }
    mode = 0;
    captureRaw();
    break;
    case CMD_RESEND_RAW:
    temp = 0xff;
    if (!g_rawFrameReady) { Serial.println(F("ACK CMD No RAW frame to resend END")); break; }
    sendRawFrame(true);
    break;
    case 0x20:
    mode = 2;
    temp = 0xff;
    start_capture = 2;
    Serial.println(F("ACK CMD CAM start video streaming. END"));
    break;
    case 0x30:
    mode = 3;
    temp = 0xff;
    start_capture = 3;
    Serial.println(F("ACK CMD CAM start single shoot. END"));
    break;
    case 0x31:
    temp = 0xff;
    g_rawMode = false;
    g_rawFrameReady = false;
    myCAM.set_format(BMP);
    myCAM.InitCAM();
    myCAM.clear_bit(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
    myCAM.wrSensorReg16_8(0x3818, 0x81);
    myCAM.wrSensorReg16_8(0x3621, 0xA7);
    break;
    default:
    handleSensorCommand(temp);
    break;
  }
}
if (mode == 1)
{
  if (start_capture == 1)
  {
    myCAM.flush_fifo();
    myCAM.clear_fifo_flag();
    //Start capture
    myCAM.start_capture();
    start_capture = 0;
  }
  if (myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK))
  {
    Serial.println(F("ACK CMD CAM Capture Done. END"));delay(50);
    read_fifo_burst(myCAM);
    //Clear the capture done flag
    myCAM.clear_fifo_flag();
  }
}
else if (mode == 2)
{
  while (1)
  {
    temp = Serial.read();
    if (temp == 0x21)
    {
      start_capture = 0;
      mode = 0;
      Serial.println(F("ACK CMD CAM stop video streaming. END"));
      break;
    }
    handleSensorCommand(temp);
    if (start_capture == 2)
    {
      myCAM.flush_fifo();
      myCAM.clear_fifo_flag();
      //Start capture
      myCAM.start_capture();
      start_capture = 0;
    }
    if (myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK))
    {
      uint32_t length = 0;
      length = myCAM.read_fifo_length();
      if ((length >= MAX_FIFO_SIZE) | (length == 0))
      {
        myCAM.clear_fifo_flag();
        start_capture = 2;
        continue;
      }
      myCAM.CS_LOW();
      myCAM.set_fifo_burst();//Set fifo burst mode
      temp =  SPI.transfer(0x00);
      length --;
      while ( length-- )
      {
        temp_last = temp;
        temp =  SPI.transfer(0x00);
        if (is_header == true)
        {
          Serial.write(temp);
        }
        else if ((temp == 0xD8) & (temp_last == 0xFF))
        {
          is_header = true;
          Serial.println(F("ACK IMG END"));
          Serial.write(temp_last);
          Serial.write(temp);
        }
        if ( (temp == 0xD9) && (temp_last == 0xFF) ) //If find the end ,break while,
        break;
        delayMicroseconds(4);
      }
      myCAM.CS_HIGH();
      myCAM.clear_fifo_flag();
      start_capture = 2;
      is_header = false;
    }
  }
}
else if (mode == 3)
{
  if (start_capture == 3)
  {
    //Flush the FIFO
    myCAM.flush_fifo();
    myCAM.clear_fifo_flag();
    //Start capture
    myCAM.start_capture();
    start_capture = 0;
  }
  if (myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK))
  {
    Serial.println(F("ACK CMD CAM Capture Done. END"));delay(50);
    uint8_t temp, temp_last;
    uint32_t length = 0;
    length = myCAM.read_fifo_length();
    if (length >= MAX_FIFO_SIZE )
    {
      Serial.println(F("ACK CMD Over size. END"));
      myCAM.clear_fifo_flag();
      return;
    }
    if (length == 0 ) //0 kb
    {
      Serial.println(F("ACK CMD Size is 0. END"));
      myCAM.clear_fifo_flag();
      return;
    }
    myCAM.CS_LOW();
    myCAM.set_fifo_burst();//Set fifo burst mode

    Serial.write(0xFF);
    Serial.write(0xAA);
    for (temp = 0; temp < BMPIMAGEOFFSET; temp++)
    {
      Serial.write(pgm_read_byte(&bmp_header[temp]));
    }
    //SPI.transfer(0x00);
    char VH, VL;
    int i = 0, j = 0;
    for (i = 0; i < 240; i++)
    {
      for (j = 0; j < 320; j++)
      {
        VH = SPI.transfer(0x00);;
        VL = SPI.transfer(0x00);;
        Serial.write(VL);
        delayMicroseconds(12);
        Serial.write(VH);
        delayMicroseconds(12);
      }
    }
    Serial.write(0xBB);
    Serial.write(0xCC);
    myCAM.CS_HIGH();
    //Clear the capture done flag
    myCAM.clear_fifo_flag();
  }
}
}
uint8_t read_fifo_burst(ArduCAM myCAM)
{
  uint8_t temp = 0, temp_last = 0;
  uint32_t length = 0;
  length = myCAM.read_fifo_length();
  Serial.println(length, DEC);
  if (length >= MAX_FIFO_SIZE) //512 kb
  {
    Serial.println(F("ACK CMD Over size. END"));
    return 0;
  }
  if (length == 0 ) //0 kb
  {
    Serial.println(F("ACK CMD Size is 0. END"));
    return 0;
  }
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();//Set fifo burst mode
  temp =  SPI.transfer(0x00);
  length --;
  while ( length-- )
  {
    temp_last = temp;
    temp =  SPI.transfer(0x00);
    if (is_header == true)
    {
      Serial.write(temp);
    }
    else if ((temp == 0xD8) & (temp_last == 0xFF))
    {
      is_header = true;
      Serial.println(F("ACK IMG END"));
      Serial.write(temp_last);
      Serial.write(temp);
    }
    if ( (temp == 0xD9) && (temp_last == 0xFF) ) //If find the end ,break while,
    break;
    delayMicroseconds(15);
  }
  myCAM.CS_HIGH();
  is_header = false;
  return 1;
}
