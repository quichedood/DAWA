/**************************************************************
  DAWA 5.0 (Arduino M0)
  Triumph motorbikes laptimer/datalogger
  Edouard PIGEON - 2017
**************************************************************/

// Librairies
#include <SPI.h>

/**************************************************************
  Time library
  https://github.com/PaulStoffregen/Time
  Please rename in "TimeLib.h" var "DAYS_PER_WEEK" to "DAYS_PER_WEEK_ALT" as the same var is already used in NeoGPS library
**************************************************************/
#include <Time.h>
#include <TimeLib.h>

/**************************************************************
  NeoGPS library
  https://github.com/SlashDevin/NeoGPS
**************************************************************/
#include <NMEAGPS.h>
HardwareSerial & GPS_PORT = Serial5;
#define DEBUG_PORT SERIAL_PORT_USBVIRTUAL // Debug to USB serial console
static NMEAGPS  gps;
static gps_fix  fix_data;
static gps_fix  fix_data_prev;

/**************************************************************
  thomasfredericks/Bounce2 library
  https://github.com/thomasfredericks/Bounce2
**************************************************************/
#include <Bounce2.h>
Bounce debouncer = Bounce();

/**************************************************************
  Greiman/SdFat library
  https://github.com/greiman/SdFat
**************************************************************/
#include <SdFat.h>

/**************************************************************
  Modified dtostrf function defnition
  http://forum.arduino.cc/index.php?topic=368720.0
**************************************************************/
#include <dtostrf.h>

/**************************************************************
  External EEPROM library

  extEEPROM.c >
  //REMOVE l.98 :
  //TWBR = ( (F_CPU / twiFreq) - 16) / 2;

  //REPLACE l.98 :
  //sercom3.disableWIRE();                         // Disable the I2C bus
  //SERCOM3->I2CM.BAUD.bit.BAUD = SystemCoreClock / ( 2 * 400000) - 1 ;   // // Set the I2C SCL frequency to 400kHz
  //sercom3.enableWIRE();                          // Restart the I2C bus
  // https://forum.arduino.cc/index.php?topic=347425.0

  extEEPROM.h >
  ADD l.63 :
  #define BUFFER_LENGTH 64 // Add var here because no more defined in wire.h (SAMD21's buffer = 64)
**************************************************************/
#include <extEEPROM.h> // Arduino External EEPROM Library v3.0 (Official Github : https://github.com/JChristensen/extEEPROM)
extEEPROM eep(kbits_2, 1, 8); // I2C Address 0x50

/**************************************************************
  Modification of the EEPROMAnything library that allows you to read and write any data structure or array to and from the external EEPROM
  https://forum.arduino.cc/index.php?topic=358648.0
**************************************************************/
#include <EEPROMAnything2.h> // Enable object read/write on EEPROM

/**************************************************************
  LCD library
  SPI ASCII OLED : https://github.com/greiman/SSD1306Ascii
**************************************************************/
// SPI ASCII OLED libraries
#include <SSD1306Ascii.h> // OLED LCD based on SSD1306 chip, ASCII only no graphics
#include <SSD1306AsciiSpi.h>
SSD1306AsciiSpi OLED_PORT; // 128x64 LCD OLED
#define OLED_DC 9
#define OLED_CS 13

/**************************************************************
  9-axis Bosch BNO055 Adafruit library
  https://github.com/adafruit/Adafruit_BNO055
**************************************************************/
#include <Adafruit_BNO055.h> // Adafruit 9-axis BNO_055 lib
#include <Adafruit_Sensor.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS_B); // USE_BNO055_ADDRESS_B = Adress 0x29 (COM3 pin set to 1)

/**************************************************************
  Bluetooth (HM11 - CC2541)
  http://jnhuamao.cn/bluetooth.asp?id=1
  If you get counterfeit HM-11, some commands won't probably work. Try flash it with original firmware :
  https://forum.arduino.cc/index.php?topic=393655.0
**************************************************************/
#include <wiring_private.h> // pinPeripheral() function
Uart BLUETOOTH_PORT (&sercom5, 7, 6, SERCOM_RX_PAD_3, UART_TX_PAD_2); //Disable SERCOM5 in <APPDATA>\local\Arduino15\packages\arduino\hardware\samd\1.6.14\variants\arduino_mzero\variant.cpp (Serial + Sercom5)

/**************************************************************
  Variable PINs I/O
**************************************************************/
#define CSV_DELIM ';'
const float bikeRpmCoeff = 1.05; // Enable high RPM correction
const unsigned int maxTrackDistance = 5; // Autoselect nearer track (unit = km)
const uint8_t ledPin = 4; // D4
const uint8_t powerPin = A3; //A3
const uint8_t brakePin = 11; // D11
const uint8_t throttlePin = A0; // A0
const uint8_t gearPin = A1; // A1
const uint8_t sdCsPin = 5; // D5 (was D7 in previous versions)
const uint8_t oledResetPin = 8; // D8

// GPS Configuration - General
const unsigned char ubxRate10Hz[] PROGMEM = {0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00};
const unsigned char ubxTimepulse[] PROGMEM = {0x06, 0x31, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29, 0x00, 0x00, 0x00};
//                                           |ID         |Lenght     |TP   |res  |res        |antCableD  |rfGrDelay  |freqPeriod             |freqPeriod lock        |Pulselenghtratio       |Pulselenghtratiolock   |UserConfigDelay        |Flags                 |
const unsigned char ubxPrtConf[] PROGMEM = {0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xd0, 0x08, 0x00, 0x00, 0x00, 0xc2, 0x01, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00}; // 115200bps
//                                         |ID         |Lenght     |Port |res  |TX Ready   |mode                   |baudrate               |inPrMask   |outPrMask  |flags      |res       |

// GPS Configuration - Enable/disable specific NMEA sentences
const unsigned char ubxEnableRMC[] PROGMEM = {0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
const unsigned char ubxDisableGLL[] PROGMEM = {0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const unsigned char ubxDisableGSA[] PROGMEM = {0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const unsigned char ubxDisableGSV[] PROGMEM = {0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const unsigned char ubxDisableVTG[] PROGMEM = {0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const unsigned char ubxDisableZDA[] PROGMEM = {0x06, 0x01, 0x08, 0x00, 0xF0, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// GPS Configuration - Backup configuration to non volatile memory
// const unsigned char ubxSave[] PROGMEM = {0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x01}; // No EEPROM => Backup battery

/**************************************************************
  Vars
**************************************************************/
unsigned long lastPinRead = 0;
unsigned long lastLCDupdate = 0;
unsigned long lastSdSync = 0;
unsigned long fixCount = 0;
unsigned int elapsedTime = 0;
char trackName[16];
bool recordTrackData;
byte lapCounter = 0;
bool addFinishLog = 0;

// GPS & Timing
float rescaleGPS = 10000000.0; // We use "long" for GPS coordinates to keep precision ("float" on Arduino have only 6 decimal digits of precision) ### https://gis.stackexchange.com/questions/8650/measuring-accuracy-of-latitude-and-longitude
bool gpsDataReady = false;
long flineLat1, flineLon1, flineLat2, flineLon2 = 0; // Finish line GPS coordinates
long posCrossLat, posCrossLon;
float coordsDistance, totalDistance;
float tToFl; // Time To Finish Line
int timeCsec, timeSec, lastFlSec, lastFlCsec, lapSec, lapCsec, runMinutes, runSeconds;
short unsigned int trackId, histTrackId, lapId;
long histTimestamp;
char histDate[14], lapTime[10];
float lapTimeSec, bestLapTime;

// Sensors
unsigned int bikeRpm;
unsigned int bikeSpeed;
unsigned int gearValue;
char gear;
byte gearNCheck = 0;
bool brake;
bool isRunning = false;
unsigned int throttle;

// SD Card
SdFat sd;
File logFile; // One line every 100ms with all detailed data
File lapFile; // One line per lap with laptime
File trackFile; // One line per track with GPS coordinates of finish line
File historyFile; // History of all sessions
char filename[32];

// 9-axis sensor
byte temperature; // Don't go out when freezing ;)
byte cal_sys, cal_gyro, cal_accel, cal_mag = 0;
adafruit_bno055_offsets_t calibrationData;
sensors_event_t event;
//byte *calibrationBuffer = (byte *) &calibrationData;

// Bluetooth
char c;
char cmdBuffer[16];
byte statId = 0;

// ADC
unsigned int an1Value, an2Value, an3Value, an4Value, an5Value, an6Value, an7Value, an8Value = 0;

/**************************************************************
  SERCOM5 : Secondary hardware serial for bluetooth
**************************************************************/
void SERCOM5_Handler() {
  BLUETOOTH_PORT.IrqHandler();
}

/**************************************************************
  Build a string character by character (serial bluetooth)
**************************************************************/
char processCharInput(char* cmdBuffer, const char c) {
  if (c >= 32 && c <= 126) { // Ignore control and special ascii characters
    if (strlen(cmdBuffer) < 16) {
      strncat(cmdBuffer, &c, 1);
    } else {
      return '\n';
    }
  }
  return c;
}

/**************************************************************
  Triggered when initialization error (program stop and slow blinking led)
**************************************************************/
void initerror() {
  delay(2000); // Pause 3 sec pour lecture infos LCD
  while (1) {
    digitalWrite(ledPin, digitalRead(ledPin) ^ 1); // Quick LED blink (there's an error)
    delay(200);
  }
}

/**************************************************************
  Send UBX commands to UBLOX GPS
**************************************************************/
void sendUBX(const unsigned char *progmemBytes, size_t len) {
  GPS_PORT.write(0xB5); // SYNC1
  GPS_PORT.write(0x62); // SYNC2

  uint8_t a = 0, b = 0;
  while (len-- > 0) {
    uint8_t c = pgm_read_byte(progmemBytes++);
    a += c;
    b += a;
    GPS_PORT.write(c);
  }
  GPS_PORT.write(a); // CHECKSUM A
  GPS_PORT.write(b); // CHECKSUM B
}

/**************************************************************
  Read CSV file
  https://github.com/greiman/SdFat/blob/master/examples/ReadCsv/ReadCsv.ino

  Read a CSV file one field at a time.
  file - File to read.
  str - Character array for the field.
  size - Size of str array.
  delim - csv delimiter.
  return - negative value for failure.
        delimiter, '\n' or zero(EOF) for success.
**************************************************************/
int csvReadText(File* file, char* str, size_t size, char delim) {
  char ch;
  int rtn;
  size_t n = 0;
  while (true) {
    // check for EOF
    if (!file->available()) {
      rtn = 0;
      break;
    }
    if (file->read(&ch, 1) != 1) {
      // read error
      rtn = -1;
      break;
    }
    // Delete CR.
    if (ch == '\r') {
      continue;
    }
    if (ch == delim || ch == '\n') {
      rtn = ch;
      break;
    }
    if ((n + 1) >= size) {
      // string too long
      rtn = -2;
      n--;
      break;
    }
    str[n++] = ch;
  }
  str[n] = '\0';
  return rtn;
}

int csvReadInt32(File* file, int32_t* num, char delim) {
  char buf[20];
  char* ptr;
  int rtn = csvReadText(file, buf, sizeof(buf), delim);
  if (rtn < 0) return rtn;
  *num = strtol(buf, &ptr, 10);
  if (buf == ptr) return -3;
  while (isspace(*ptr)) ptr++;
  return *ptr == 0 ? rtn : -4;
}

int csvReadInt16(File* file, int16_t* num, char delim) {
  int32_t tmp;
  int rtn = csvReadInt32(file, &tmp, delim);
  if (rtn < 0) return rtn;
  if (tmp < INT_MIN || tmp > INT_MAX) return -5;
  *num = tmp;
  return rtn;
}

int csvReadUint32(File* file, uint32_t* num, char delim) {
  char buf[20];
  char* ptr;
  int rtn = csvReadText(file, buf, sizeof(buf), delim);
  if (rtn < 0) return rtn;
  *num = strtoul(buf, &ptr, 10);
  if (buf == ptr) return -3;
  while (isspace(*ptr)) ptr++;
  return *ptr == 0 ? rtn : -4;
}

int csvReadUint16(File* file, uint16_t* num, char delim) {
  uint32_t tmp;
  int rtn = csvReadUint32(file, &tmp, delim);
  if (rtn < 0) return rtn;
  if (tmp > UINT_MAX) return -5;
  *num = tmp;
  return rtn;
}

int csvReadDouble(File* file, double* num, char delim) {
  char buf[20];
  char* ptr;
  int rtn = csvReadText(file, buf, sizeof(buf), delim);
  if (rtn < 0) return rtn;
  *num = strtod(buf, &ptr);
  if (buf == ptr) return -3;
  while (isspace(*ptr)) ptr++;
  return *ptr == 0 ? rtn : -4;
}

int csvReadFloat(File* file, float* num, char delim) {
  double tmp;
  int rtn = csvReadDouble(file, &tmp, delim);
  if (rtn < 0)return rtn;
  // could test for too large.
  *num = tmp;
  return rtn;
}

/**************************************************************
  Calculate 2 line segments intersection
**************************************************************/
bool segIntersect(long pos_now_lat, long pos_now_lon, long pos_prev_lat, long pos_prev_lon, long trackLat1, long trackLon1, long trackLat2, long trackLon2, long &pos_cross_lat, long &pos_cross_lon) {
  float denom, s_numer, t_numer, t;
  long track_pos_x, track_pos_y, pos_x, pos_y, trackLon, trackLat;
  bool denomPositive;

  trackLon = trackLon2 - trackLon1;
  trackLat = trackLat2 - trackLat1;
  pos_x = pos_now_lon - pos_prev_lon;
  pos_y = pos_now_lat - pos_prev_lat;
  denom = trackLon * pos_y - pos_x * trackLat;
  if (denom == 0) {
    return 0; // Collinear
  }

  if (denom > 0) {
    denomPositive = true;
  } else {
    denomPositive = false;
  }

  track_pos_x = trackLon1 - pos_prev_lon;
  track_pos_y = trackLat1 - pos_prev_lat;

  s_numer = trackLon * track_pos_y - trackLat * track_pos_x;
  if ((s_numer < 0) == denomPositive) {
    return 0; // No collision
  }

  t_numer = pos_x * track_pos_y - pos_y * track_pos_x;
  if ((t_numer < 0) == denomPositive) {
    return 0; // No collision
  }

  if (((s_numer > denom) == denomPositive) || ((t_numer > denom) == denomPositive)) {
    return 0; // No collision
  }

  // Collision detected
  t = t_numer / denom;
  pos_cross_lat = trackLat1 + (t * trackLat);
  pos_cross_lon = trackLon1 + (t * trackLon);

  return 1;
}

/**************************************************************
  Calculate distance between 2 GPS coords (unit = meters) - haversine formula
**************************************************************/
float gpsDistance(long lat1, long lon1, long lat2, long lon2) {
  float dlam, dphi, p = 0.017453292519943295; // (Pi / 180)
  dphi = p * (lat1 + lat2) * 0.5e-7; //average latitude in radians
  float cphi = cos(dphi);
  dphi = p * ( lat2 - lat1) * 1.0e-7; //differences in degrees (to radians)
  dlam = p * ( lon2 - lon1) * 1.0e-7;
  dlam *= cphi;  //correct for latitude
  return 6371000.0 * sqrt(dphi * dphi + dlam * dlam);
}

/**************************************************************
  Add a time [arg0] (Ex : 5.14) to another which is composed of seconds [arg1] and milliseconds [arg2]. Return seconds [arg3] and milliseconds [arg4]
**************************************************************/
void TimeAdd(float timeSecCsec, int endSec, int endCsec, int &returnSec, int &returnCsec) {
  returnSec = endSec + (int)(timeSecCsec + (endCsec / 100.00));
  returnCsec = ((timeSecCsec + (endCsec / 100.00)) - (int)(timeSecCsec + (endCsec / 100.00))) * 100;
}

/**************************************************************
  Substract a time [arg0][arg1] to another [arg2][arg3]. Return seconds [arg4] and milliseconds [arg6]
**************************************************************/
void TimeSubstract(int s1, int cs1, int s2, int cs2, int &returnSec, int &returnCsec) {
  returnCsec = cs1 - cs2;
  if (returnCsec < 0) {
    returnSec = (s1 - s2) - 1;
    returnCsec += 100;
  } else {
    returnSec = s1 - s2;
  }
}

/**************************************************************
  Print realtime data on one output (oled screen or bluetooth serial)
**************************************************************/
void printData(gps_fix &fix_data, Print &OUT_PORT) {
  if (fix_data.valid.date && fix_data.valid.time) {
    if (fix_data.dateTime.date < 10) OUT_PORT.print("0"); // Leading zeros
    OUT_PORT.print(fix_data.dateTime.date);
    OUT_PORT.print("/");
    if (fix_data.dateTime.month < 10) OUT_PORT.print("0"); // Leading zeros
    OUT_PORT.print(fix_data.dateTime.month);
    OUT_PORT.print("/");
    if (fix_data.dateTime.year < 10) OUT_PORT.print("0"); // Leading zeros
    OUT_PORT.print(fix_data.dateTime.year);
    OUT_PORT.print(" ");
    if (fix_data.dateTime.hours < 10) OUT_PORT.print("0"); // Leading zeros
    OUT_PORT.print(fix_data.dateTime.hours);
    OUT_PORT.print(":");
    if (fix_data.dateTime.minutes < 10) OUT_PORT.print("0"); // Leading zeros
    OUT_PORT.print(fix_data.dateTime.minutes);
    OUT_PORT.print(":");
    if (fix_data.dateTime.seconds < 10) OUT_PORT.print("0"); // Leading zeros
    OUT_PORT.println(fix_data.dateTime.seconds);
  } else {
    OUT_PORT.println("No GPS signal");
  }
  /*OUT_PORT.print("GPS:");
  OUT_PORT.print(fix_data.dateTime);
  OUT_PORT.print(".");
  if (fix_data.dateTime_cs < 10) OUT_PORT.print("0"); // Leading zeros (remember "fix_data.dateTime_cs" is an integer !!)
  OUT_PORT.println(fix_data.dateTime_cs);*/
  OUT_PORT.print("RPM:       ");
  OUT_PORT.println(bikeRpm);
  OUT_PORT.print("SPEED:     ");
  OUT_PORT.println(bikeSpeed);
  OUT_PORT.print("GEAR:      ");
  OUT_PORT.println(gear);
  OUT_PORT.print("BRAKE:     ");
  OUT_PORT.println(brake);
  OUT_PORT.print("THROTTLE:  ");
  OUT_PORT.println(throttle);
  //OUT_PORT.print("TEMP:      ");
  //OUT_PORT.println(temperature);
  //OUT_PORT.print("ADC1:      ");
  if (an1Value < 10) OUT_PORT.print("000");
  if (an1Value >= 10 && an1Value < 100) OUT_PORT.print("00");
  if (an1Value >= 100 && an1Value < 1000) OUT_PORT.print("0");
  OUT_PORT.print(an1Value);
  OUT_PORT.print("-");
    OUT_PORT.print(an2Value);
    OUT_PORT.print("-");
    OUT_PORT.print(an3Value);
    OUT_PORT.print("-");
    OUT_PORT.println(an4Value);
    OUT_PORT.print(an5Value);
    OUT_PORT.print("-");
    OUT_PORT.print(an6Value);
    OUT_PORT.print("-");
    OUT_PORT.print(an7Value);
    OUT_PORT.print("-");
    OUT_PORT.println(an8Value);
}

/**************************************************************
  Initialize ADC (I2C)
**************************************************************/
byte initADC() {
  Wire.beginTransmission(0x1D);
  Wire.write(byte(0x0B)); // Advanced Configuration Register
  Wire.write(byte(0x03)); // 00000011 (Vref ext & mode = 1)
  Wire.endTransmission();

  Wire.beginTransmission(0x1D);
  Wire.write(byte(0x07)); // Conversion Rate Register
  Wire.write(byte(0x01)); // 00000001 (Conversion mode = continuous)
  Wire.endTransmission();

  Wire.beginTransmission(0x1D);
  Wire.write(byte(0x08)); // Channel Disable Register
  Wire.write(byte(0b00001111)); // Enable or disable channels
  Wire.endTransmission();

  Wire.beginTransmission(0x1D);
  Wire.write(byte(0x00)); // Configuration Register
  Wire.write(byte(0x01)); // 00000001 (Start conversion)
  return Wire.endTransmission();
}

/**************************************************************
  Read all ADC values (8)
**************************************************************/
void readAdcValues(unsigned int &an1Value, unsigned int &an2Value, unsigned int &an3Value, unsigned int &an4Value, unsigned int &an5Value, unsigned int &an6Value, unsigned int &an7Value, unsigned int &an8Value) {
  Wire.beginTransmission(0x1D); // Talk to ADC128D818 - A0 and A1 to GND
  Wire.write(byte(0x20)); // Busy Status Register (Bit 1 = Not Ready > Waiting for the power-up sequence to end)
  Wire.endTransmission();
  Wire.requestFrom(0x1D, 2); // Read Register 0x20
  if (Wire.available() == 2) {
    an1Value = Wire.read() << 4 | Wire.read(); // We read a 12 bits value (2x 8 bits I2C reads)
  }

  Wire.beginTransmission(0x1D); // Talk to ADC128D818 - A0 and A1 to GND
  Wire.write(byte(0x21)); // Busy Status Register (Bit 1 = Not Ready > Waiting for the power-up sequence to end)
  Wire.endTransmission();
  Wire.requestFrom(0x1D, 2); // Read Register 0x21
  if (Wire.available() == 2) {
    an2Value = Wire.read() << 4 | Wire.read(); // We read a 12 bits value (2x 8 bits I2C reads)
  }

  Wire.beginTransmission(0x1D); // Talk to ADC128D818 - A0 and A1 to GND
  Wire.write(byte(0x22)); // Busy Status Register (Bit 1 = Not Ready > Waiting for the power-up sequence to end)
  Wire.endTransmission();
  Wire.requestFrom(0x1D, 2); // Read Register 0x22
  if (Wire.available() == 2) {
    an3Value = Wire.read() << 4 | Wire.read(); // We read a 12 bits value (2x 8 bits I2C reads)
  }

  Wire.beginTransmission(0x1D); // Talk to ADC128D818 - A0 and A1 to GND
  Wire.write(byte(0x23)); // Busy Status Register (Bit 1 = Not Ready > Waiting for the power-up sequence to end)
  Wire.endTransmission();
  Wire.requestFrom(0x1D, 2); // Read Register 0x23
  if (Wire.available() == 2) {
    an4Value = Wire.read() << 4 | Wire.read(); // We read a 12 bits value (2x 8 bits I2C reads)
  }

  Wire.beginTransmission(0x1D); // Talk to ADC128D818 - A0 and A1 to GND
  Wire.write(byte(0x24)); // Busy Status Register (Bit 1 = Not Ready > Waiting for the power-up sequence to end)
  Wire.endTransmission();
  Wire.requestFrom(0x1D, 2); // Read Register 0x24
  if (Wire.available() == 2) {
    an5Value = Wire.read() << 4 | Wire.read(); // We read a 12 bits value (2x 8 bits I2C reads)
  }

  Wire.beginTransmission(0x1D); // Talk to ADC128D818 - A0 and A1 to GND
  Wire.write(byte(0x25)); // Busy Status Register (Bit 1 = Not Ready > Waiting for the power-up sequence to end)
  Wire.endTransmission();
  Wire.requestFrom(0x1D, 2); // Read Register 0x25
  if (Wire.available() == 2) {
    an6Value = Wire.read() << 4 | Wire.read(); // We read a 12 bits value (2x 8 bits I2C reads)
  }

  Wire.beginTransmission(0x1D); // Talk to ADC128D818 - A0 and A1 to GND
  Wire.write(byte(0x26)); // Busy Status Register (Bit 1 = Not Ready > Waiting for the power-up sequence to end)
  Wire.endTransmission();
  Wire.requestFrom(0x1D, 2); // Read Register 0x26
  if (Wire.available() == 2) {
    an7Value = Wire.read() << 4 | Wire.read(); // We read a 12 bits value (2x 8 bits I2C reads)
  }

  Wire.beginTransmission(0x1D); // Talk to ADC128D818 - A0 and A1 to GND
  Wire.write(byte(0x27)); // Busy Status Register (Bit 1 = Not Ready > Waiting for the power-up sequence to end)
  Wire.endTransmission();
  Wire.requestFrom(0x1D, 2); // Read Register 0x27
  if (Wire.available() == 2) {
    an8Value = Wire.read() << 4 | Wire.read(); // We read a 12 bits value (2x 8 bits I2C reads)
  }
}

/**************************************************************
  Initialisation (I/O, BLUETOOTH, LCD, SD, DATE, 9-AXIS SENSOR, TIMERS/COUNTERS)
**************************************************************/
void setup() {
  Wire.begin(); // I2C bus init

  // Init I/O pins
  pinMode(ledPin, OUTPUT); // Button light
  pinMode(powerPin, INPUT); // ON/OFF button
  pinMode(brakePin, INPUT); // Brake sensor (0v/12v)
  pinMode(throttlePin, INPUT); // Throttle sensor (analogic from 0v to 5v)
  pinMode(gearPin, INPUT); // Gear sensor (analogic from 0v to 5v)
  pinMode(oledResetPin, OUTPUT); // Reset on OLED screen

  // Debounce power button
  debouncer.attach(powerPin);
  debouncer.interval(200); // interval in ms

  // OLED Screen Init (SPI)
  digitalWrite(oledResetPin, HIGH); // Reset pin on OLED screen should be tied to +VCC
  OLED_PORT.begin(&Adafruit128x64, OLED_CS, OLED_DC); OLED_PORT.setFont(System5x7); // SPI or I2C
  delay(1000); // LCD init delay
  OLED_PORT.clear();
  OLED_PORT.setCursor(0, 0); // Set cursor upper-left

  // Bluetooth Init (Serial2)
  OLED_PORT.print("BLUETOOTH:");
  BLUETOOTH_PORT.begin(9600);
  pinPeripheral(6, PIO_SERCOM); // Pin D6 for RX
  pinPeripheral(7, PIO_SERCOM); // Pin D7 for TX
  // ADD A TRUE BLUETOOTH TEST HERE !!
  OLED_PORT.println("         OK");
  BLUETOOTH_PORT.println("D.A.W.A. Initializing ...");
  BLUETOOTH_PORT.println("BLUETOOTH : OK");

  // GPS Init (Serial5 - Default on Arduino m0)
  OLED_PORT.print("GPS:");
  BLUETOOTH_PORT.print("GPS : ");
  GPS_PORT.begin(9600); // Start the UART @9600bps for the GPS device (default speed)
  sendUBX(ubxPrtConf, sizeof(ubxPrtConf)); // Set UART speed to 115200bps (Warning : @9600bps > ~5sec delay on GPS data)
  delay(100);
  GPS_PORT.end();
  GPS_PORT.begin(115200); // Start the UART @115200bps
  sendUBX(ubxRate10Hz, sizeof(ubxRate10Hz)); // Set refresh rate to 10Hz
  sendUBX(ubxTimepulse, sizeof(ubxTimepulse)); // Set timepulse output ON
  sendUBX(ubxEnableRMC, sizeof(ubxEnableRMC)); // Enable RMC trames, disable all others
  sendUBX(ubxDisableGLL, sizeof(ubxDisableGLL));
  sendUBX(ubxDisableGSA, sizeof(ubxDisableGSA));
  sendUBX(ubxDisableGSV, sizeof(ubxDisableGSV));
  sendUBX(ubxDisableVTG, sizeof(ubxDisableVTG));
  sendUBX(ubxDisableZDA, sizeof(ubxDisableZDA));
  //sendUBX(ubxSave, sizeof(ubxSave));
  OLED_PORT.println("               OK");
  BLUETOOTH_PORT.println("OK");

  // ADC Init (I2C Address : 0x1D)
  OLED_PORT.print("ADC:");
  BLUETOOTH_PORT.print("ADC : ");
  if (!initADC() == 0) {
    OLED_PORT.println("           FAILED");
    BLUETOOTH_PORT.println("FAILED");
    initerror();
  } else {
    OLED_PORT.println("               OK");
    BLUETOOTH_PORT.println("OK");
  }

  // EEPROM Init (I2C)
  OLED_PORT.print("EEPROM:");
  BLUETOOTH_PORT.print("EEPROM : ");
  if (!eep.begin(twiClock400kHz) == 0) {
    OLED_PORT.println("        FAILED");
    BLUETOOTH_PORT.println("FAILED");
    initerror();
  } else {
    OLED_PORT.println("            OK");
    BLUETOOTH_PORT.println("OK");
  }

  // SD card Init (SPI)
  OLED_PORT.print("SD:");
  BLUETOOTH_PORT.print("SD : ");
  if (!sd.begin(sdCsPin, SD_SCK_MHZ(50))) {
    OLED_PORT.println("            FAILED");
    BLUETOOTH_PORT.println("FAILED");
    initerror();
  } else {
    OLED_PORT.println("                OK");
    BLUETOOTH_PORT.println("OK");
  }

  // 9-axis sensor Init (I2C)
  OLED_PORT.print("9-AXIS:");
  BLUETOOTH_PORT.print("9-AXIS : ");
  if (!bno.begin()) {
    OLED_PORT.println("        FAILED");
    BLUETOOTH_PORT.println("FAILED");
    initerror();
  } else {
    OLED_PORT.println("            OK");
    BLUETOOTH_PORT.println("OK");
    bno.setExtCrystalUse(true); // Use external quartz for 9-axis sensor
    OLED_PORT.println(">> PRESS TO CALIB. <<");
    BLUETOOTH_PORT.println(">> PRESS TO CALIB. <<");
    digitalWrite(ledPin, HIGH); // We light up the led for the user (calibration is possible)
    delay(2000); // LCD user read time
    if (digitalRead(A3) == 1) { // If user press button, we start calibration
      OLED_PORT.print("CALIBRATE:");
      while (!bno.isFullyCalibrated()) { // We wait until sensor fully calibrated !! Offsets are returned only if sensor is calibrated !!
        delay(200);
        bno.getCalibration(&cal_sys, &cal_gyro, &cal_accel, &cal_mag); // Get calibration state (from 0 to 3 for the 3 sensors + main sensor calibration)
        OLED_PORT.setCursor(84, 4);
        OLED_PORT.print(cal_sys);
        OLED_PORT.print(";");
        OLED_PORT.print(cal_gyro);
        OLED_PORT.print(";");
        OLED_PORT.print(cal_accel);
        OLED_PORT.print(";");
        OLED_PORT.print(cal_mag);
        delay(200);
      }
      bno.getSensorOffsets(calibrationData); // Sensor is calibrated, we save offsets to var (calibrationData)
      if (EEPROM_writeAnything(0, calibrationData) == sizeof(adafruit_bno055_offsets_t)) { // We save the 11 offsets to eeprom
        OLED_PORT.println("  OK");
      } else {
        initerror();
      }
    } else { // Calibration not asked
      OLED_PORT.print("LOAD CALIB.");
      BLUETOOTH_PORT.print("LOAD CALIBRATION : ");
      if (EEPROM_readAnything(0, calibrationData) == sizeof(adafruit_bno055_offsets_t)) { // We try loading offsets from EEPROM
        bno.setSensorOffsets(calibrationData); // We set them in var
        OLED_PORT.println("        OK");
        BLUETOOTH_PORT.println("OK");
      } else {
        OLED_PORT.println("   NO DATA");
        BLUETOOTH_PORT.println("NO DATA");
        initerror(); // Calibration not asked and no offset in EEPROM > can't go further !
      }
    }
    digitalWrite(ledPin, LOW); // Calibration is done, LED off
  }

  // End Init
  OLED_PORT.print("              READY !");
  BLUETOOTH_PORT.println("READY !");
  BLUETOOTH_PORT.println(" ");
  OLED_PORT.clear();

  /**************************************************************
    2 counters (TCC0 & TCC1)
    for SPEED and RPM values

    m0 pinout : https://github.com/arduino/ArduinoCore-samd/blob/master/variants/arduino_mzero/variant.cpp
    Big thanks to : https://forum.arduino.cc/index.php?topic=396804.45
  **************************************************************/
  REG_PM_APBCMASK |=  PM_APBCMASK_EVSYS;    // Switch on the event system peripheral
  PM->APBCMASK.reg |= PM_APBCMASK_TCC0;     // Enable TCC0 Bus clock (Timer counter control clock)
  PM->APBCMASK.reg |= PM_APBCMASK_TCC1;     // Enable TCC1 Bus clock (Timer counter control clock)

  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable the generic clock...
                     GCLK_CLKCTRL_GEN_GCLK0 |     // .... on GCLK0...
                     GCLK_CLKCTRL_ID_EIC;         // ... to feed the GCLK0 to EIC peripheral
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable the generic clock...
                     GCLK_CLKCTRL_GEN_GCLK0 |     // ....on GCLK0...
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // ... to feed the GCLK5 to TCC0 and TCC1 peripheral
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  PORT->Group[PORTA].PMUX[19 >> 1].reg |= PORT_PMUX_PMUXO_A;     // Connect PA19 (pin 12 on m0) to peripheral A (EXTINT[3])
  PORT->Group[PORTA].PINCFG[19].reg |= PORT_PINCFG_PMUXEN;       // Enable pin peripheral multiplexation

  PORT->Group[PORTA].PMUX[18 >> 1].reg |= PORT_PMUX_PMUXO_A;     // Connect PA18 (pin 10 on m0) to peripheral A (EXTINT[3])
  PORT->Group[PORTA].PINCFG[18].reg |= PORT_PINCFG_PMUXEN;       // Enable pin peripheral multiplexation

  REG_EIC_EVCTRL |= EIC_EVCTRL_EXTINTEO3;         // Enable event from pin on external interrupt 3 (EXTINT03)
  REG_EIC_CONFIG0 |= EIC_CONFIG_SENSE3_RISE;      // Set event on rising edge of signal

  REG_EIC_EVCTRL |= EIC_EVCTRL_EXTINTEO2;         // Enable event from pin on external interrupt 2 (EXTINT02)
  REG_EIC_CONFIG0 |= EIC_CONFIG_SENSE2_RISE;      // Set event on rising edge of signal

  REG_EIC_CTRL |= EIC_CTRL_ENABLE;                // Enable EIC peripheral
  while (EIC->STATUS.bit.SYNCBUSY);               // Wait for synchronization

  // EVSYS Configuration
  REG_EVSYS_USER = EVSYS_USER_CHANNEL(1) |                                // Attach the event user (receiver) to channel n=0 (n + 1)
                   EVSYS_USER_USER(EVSYS_ID_USER_TCC0_EV_0);              // Set the event user (receiver) as timer TCC0, event 1

  REG_EVSYS_USER = EVSYS_USER_CHANNEL(2) |                                // Attach the event user (receiver) to channel n=0 (n + 1)
                   EVSYS_USER_USER(EVSYS_ID_USER_TCC1_EV_0);              // Set the event user (receiver) as timer TCC0, event 1

  REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |                // No event output edge detection
                      EVSYS_CHANNEL_PATH_ASYNCHRONOUS |                   // Set event path as asynchronous
                      EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_3) |    // Set event generator (sender) as external interrupt 3
                      EVSYS_CHANNEL_CHANNEL(0);                           // Attach the generator (sender) to channel 0

  REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |                // No event output edge detection
                      EVSYS_CHANNEL_PATH_ASYNCHRONOUS |                   // Set event path as asynchronous
                      EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_2) |    // Set event generator (sender) as external interrupt 3
                      EVSYS_CHANNEL_CHANNEL(1);                           // Attach the generator (sender) to channel 0


  // TCC0 & TCC1 Configuration
  REG_TCC0_CTRLA &= ~TCC_CTRLA_ENABLE;            // Disable TCC0 peripheral
  REG_TCC1_CTRLA &= ~TCC_CTRLA_ENABLE;            // Disable TCC1 peripheral

  REG_TCC0_CTRLBCLR |= TCC_CTRLBCLR_DIR;          // Clear DIR bit to count up
  while (TCC0->SYNCBUSY.bit.CTRLB);               // Wait for (write) synchronization
  REG_TCC1_CTRLBCLR |= TCC_CTRLBCLR_DIR;          // Clear DIR bit to count up
  while (TCC1->SYNCBUSY.bit.CTRLB);               // Wait for (write) synchronization

  REG_TCC0_EVCTRL |= TCC_EVCTRL_TCEI0 |           // Enable the TCC event 0 input
                     TCC_EVCTRL_EVACT0_COUNT;     // Set up TCC timer/counter to count on event

  REG_TCC0_CTRLA |= TCC_CTRLA_ENABLE;             // Enable TCC0
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization

  REG_TCC1_EVCTRL |= TCC_EVCTRL_TCEI0 |           // Enable the TCC event 0 input
                     TCC_EVCTRL_EVACT0_COUNT;     // Set up TCC timer/counter to count on event

  REG_TCC1_CTRLA |= TCC_CTRLA_ENABLE;             // Enable TCC0
  while (TCC1->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
}

/**************************************************************
  MAIN LOOP
**************************************************************/
void loop() {
  // Update the Bounce instance :
  debouncer.update();

  /**************************************************************
    Action on start/stop button
  **************************************************************/
  if (debouncer.rose()) { // long press on power button
    if (isRunning) { // If laptimer is running ...
      isRunning = false; // ... we stop recording
      digitalWrite(ledPin, 0); // LED off
      logFile.close(); // Close file on SDcard
      lapFile.close(); // Close file on SDcard
      OLED_PORT.clear();
      BLUETOOTH_PORT.println("[Session stopped !]");
    } else { // If laptimer is NOT running ...
      OLED_PORT.clear();
      OLED_PORT.println("Running ...");
      BLUETOOTH_PORT.println("[Start new session !]");

      /**************************************************************
        Select nearer track from the file "TRACKS.csv" on sdcard
      **************************************************************/
      recordTrackData = false;
      trackFile = sd.open("TRACKS.csv", FILE_READ);
      if (trackFile) {
        while (trackFile.available()) {
          csvReadUint16(&trackFile, &trackId, CSV_DELIM);
          csvReadText(&trackFile, trackName, sizeof(trackName), CSV_DELIM); // One line per track : "CAROLE;48.979993;2.522435;48.980023;2.522633" (<trackname>;<startline_a_lat>;<startline_a_lon>;<startline_b_lat>;<startline_b_lon>)
          csvReadInt32(&trackFile, &flineLat1, CSV_DELIM);                  // Points A & B should be at the left and at the right of the finishline (a few meters)
          csvReadInt32(&trackFile, &flineLon1, CSV_DELIM);
          csvReadInt32(&trackFile, &flineLat2, CSV_DELIM);
          csvReadInt32(&trackFile, &flineLon2, CSV_DELIM);

          // Calculate distance between 2 GPS coordinates
          coordsDistance = gpsDistance(fix_data.latitudeL(), fix_data.longitudeL(), flineLat1, flineLon1) / 1000;

          if (coordsDistance <= maxTrackDistance) {
            recordTrackData = true;
            OLED_PORT.print(trackName);
            OLED_PORT.print(" (");
            OLED_PORT.print(coordsDistance, 1);
            OLED_PORT.println("km)");
            BLUETOOTH_PORT.print("Track selected : ");
            BLUETOOTH_PORT.print(trackName);
            BLUETOOTH_PORT.print(" (");
            BLUETOOTH_PORT.print(coordsDistance, 1);
            BLUETOOTH_PORT.println("km)");
            break;
          }
        }
        trackFile.close();
      }
      if (recordTrackData == false) {
        OLED_PORT.println("No track file !");
        BLUETOOTH_PORT.println("No track file !");
      }

      /**************************************************************
        Create new datafiles : history file (append) + log file (create new) + laptime file (create new)
      **************************************************************/
      sprintf(filename, "HISTORY.csv");
      if (historyFile.open(filename, O_CREAT | O_APPEND | O_WRITE)) {
        historyFile.print(fix_data.dateTime);
        historyFile.print(";");
        historyFile.print(fix_data.dateTime.year);
        historyFile.print(fix_data.dateTime.month);
        historyFile.print(fix_data.dateTime.date);
        historyFile.print("-");
        if (fix_data.dateTime.hours < 10) historyFile.print("0"); // Leading zeros
        historyFile.print(fix_data.dateTime.hours);
        if (fix_data.dateTime.minutes < 10) historyFile.print("0"); // Leading zeros
        historyFile.print(fix_data.dateTime.minutes);
        if (fix_data.dateTime.seconds < 10) historyFile.print("0"); // Leading zeros
        historyFile.print(fix_data.dateTime.seconds);
        historyFile.print(";");
        historyFile.println(trackId);
        historyFile.close(); // Close file on SDcard
        BLUETOOTH_PORT.print("Append file : ");
        BLUETOOTH_PORT.println(filename);
      } else {
        initerror();
      }
      sprintf(filename, "%02u%02u%02u-%02u%02u%02u.csv", fix_data.dateTime.year, fix_data.dateTime.month, fix_data.dateTime.date, fix_data.dateTime.hours, fix_data.dateTime.minutes, fix_data.dateTime.seconds);
      if (logFile.open(filename, O_CREAT | O_WRITE | O_EXCL)) {
        logFile.println(F("Time;Distance;Lap;Brake;Throttle;Gear;RPM;KPH;KPHGPS;Heading;Roll;Pitch;Temp;Latitude;Longitude"));
        BLUETOOTH_PORT.print("Create new file : ");
        BLUETOOTH_PORT.println(filename);
      } else {
        initerror();
      }
      sprintf(filename, "%02u%02u%02u-%02u%02u%02u-LAPTIMES.csv", fix_data.dateTime.year, fix_data.dateTime.month, fix_data.dateTime.date, fix_data.dateTime.hours, fix_data.dateTime.minutes, fix_data.dateTime.seconds);
      if (lapFile.open(filename, O_CREAT | O_WRITE | O_EXCL)) {
        BLUETOOTH_PORT.print("Create new file : ");
        BLUETOOTH_PORT.println(filename);
      } else {
        initerror();
      }

      /**************************************************************
        Init some vars
      **************************************************************/
      isRunning = true; // ... we start recording
      lapCounter = 0;
      totalDistance = 0;
      addFinishLog = false;
    }
  }

  /**************************************************************
    Get bluetooth commands
    Wait for orders to show requested stats
  **************************************************************/
  while (BLUETOOTH_PORT.available()) {
    c = processCharInput(cmdBuffer, BLUETOOTH_PORT.read());
    if (c == '\n') {
      if (strcmp("#GetLastRun", cmdBuffer) == 0) {
        statId = 1;
      }
      if (strcmp("#GetToday", cmdBuffer) == 0) {
        statId = 2;
      }
      if (strcmp("#GetTrackBest", cmdBuffer) == 0) {
        statId = 3;
      }
      if (strcmp("#GetData", cmdBuffer) == 0) {
        statId = 4;
      }
      if (strcmp("#GetGnu", cmdBuffer) == 0) {
        statId = 5;
      }
      cmdBuffer[0] = 0;
    }
  }

  if (statId != 0) {
    switch (statId) {
      /**************************************************************
        #GetLastRun > Print last session
      **************************************************************/
      case 1:
        historyFile = sd.open("HISTORY.csv", FILE_READ); // Read entire history file to get last session laptime filename (based on timestamp)
        if (historyFile) {
          while (historyFile.available()) {
            csvReadInt32(&historyFile, &histTimestamp, CSV_DELIM);
            csvReadText(&historyFile, histDate, sizeof(histDate), CSV_DELIM);
            csvReadUint16(&historyFile, &histTrackId, CSV_DELIM);
          }
          historyFile.close();
          BLUETOOTH_PORT.println("----------------------------------");
          BLUETOOTH_PORT.print(day(histTimestamp));
          BLUETOOTH_PORT.print("/");
          BLUETOOTH_PORT.print(month(histTimestamp));
          BLUETOOTH_PORT.print("/");
          BLUETOOTH_PORT.print(year(histTimestamp));
          BLUETOOTH_PORT.print(" ");
          if (hour(histTimestamp) < 10) BLUETOOTH_PORT.print("0"); // Leading zeros
          BLUETOOTH_PORT.print(hour(histTimestamp));
          BLUETOOTH_PORT.print(":");
          if (minute(histTimestamp) < 10) BLUETOOTH_PORT.print("0"); // Leading zeros
          BLUETOOTH_PORT.print(minute(histTimestamp));
          BLUETOOTH_PORT.print(":");
          if (second(histTimestamp) < 10) BLUETOOTH_PORT.print("0"); // Leading zeros
          BLUETOOTH_PORT.println(second(histTimestamp));
          trackFile = sd.open("TRACKS.csv", FILE_READ); // Read trackfile to get track name
          if (trackFile) {
            while (trackFile.available()) {
              csvReadUint16(&trackFile, &trackId, CSV_DELIM);
              csvReadText(&trackFile, trackName, sizeof(trackName), CSV_DELIM);
              csvReadInt32(&trackFile, &flineLat1, CSV_DELIM);
              csvReadInt32(&trackFile, &flineLon1, CSV_DELIM);
              csvReadInt32(&trackFile, &flineLat2, CSV_DELIM);
              csvReadInt32(&trackFile, &flineLon2, CSV_DELIM);
              if (trackId == histTrackId) {
                trackFile.close();
                BLUETOOTH_PORT.print("Track : ");
                BLUETOOTH_PORT.println(trackName);
                break;
              }
            }
            trackFile.close();
          }
          sprintf(filename, "%s-LAPTIMES.csv", histDate);
          lapFile = sd.open(filename, FILE_READ); // Read last session laptime filename
          if (lapFile) {
            bestLapTime = 9999.00;
            while (lapFile.available()) {
              csvReadUint16(&lapFile, &lapId, CSV_DELIM);
              csvReadText(&lapFile, lapTime, sizeof(lapTime), CSV_DELIM);
              csvReadFloat(&lapFile, &lapTimeSec, CSV_DELIM);
              if (lapTimeSec < bestLapTime) {
                bestLapTime = lapTimeSec;
              }
              BLUETOOTH_PORT.print("L");
              BLUETOOTH_PORT.print(lapId);
              BLUETOOTH_PORT.print(" > ");
              BLUETOOTH_PORT.println(lapTime);
            }
            lapFile.close();
          }
        }
        BLUETOOTH_PORT.print("Best : ");
        BLUETOOTH_PORT.println(bestLapTime);
        BLUETOOTH_PORT.println("----------------------------------");
        statId = 0;
        break;

      /**************************************************************
        #GetToday > Print sessions in the last 12h
      **************************************************************/
      case 2:
        historyFile = sd.open("HISTORY.csv", FILE_READ);
        if (historyFile) {
          BLUETOOTH_PORT.println("----------------------------------");
          while (historyFile.available()) {
            csvReadInt32(&historyFile, &histTimestamp, CSV_DELIM);
            csvReadText(&historyFile, histDate, sizeof(histDate), CSV_DELIM);
            csvReadUint16(&historyFile, &trackId, CSV_DELIM);
            BLUETOOTH_PORT.print(day(histTimestamp));
            BLUETOOTH_PORT.print("/");
            BLUETOOTH_PORT.print(month(histTimestamp));
            BLUETOOTH_PORT.print("/");
            BLUETOOTH_PORT.print(year(histTimestamp));
            BLUETOOTH_PORT.print(" ");
            if (hour(histTimestamp) < 10) BLUETOOTH_PORT.print("0"); // Leading zeros
            BLUETOOTH_PORT.print(hour(histTimestamp));
            BLUETOOTH_PORT.print(":");
            if (minute(histTimestamp) < 10) BLUETOOTH_PORT.print("0"); // Leading zeros
            BLUETOOTH_PORT.print(minute(histTimestamp));
            BLUETOOTH_PORT.print(":");
            if (second(histTimestamp) < 10) BLUETOOTH_PORT.print("0"); // Leading zeros
            BLUETOOTH_PORT.println(second(histTimestamp));
            if (histTimestamp > (fix_data.dateTime - (12 * 3600))) {
              trackFile = sd.open("TRACKS.csv", FILE_READ); // Read trackfile to get track name
              if (trackFile) {
                while (trackFile.available()) {
                  csvReadUint16(&trackFile, &trackId, CSV_DELIM);
                  csvReadText(&trackFile, trackName, sizeof(trackName), CSV_DELIM);
                  csvReadInt32(&trackFile, &flineLat1, CSV_DELIM);
                  csvReadInt32(&trackFile, &flineLon1, CSV_DELIM);
                  csvReadInt32(&trackFile, &flineLat2, CSV_DELIM);
                  csvReadInt32(&trackFile, &flineLon2, CSV_DELIM);

                  if (trackId == histTrackId) {
                    trackFile.close();
                    BLUETOOTH_PORT.print("Track : ");
                    BLUETOOTH_PORT.println(trackName);
                    break;
                  }
                }
                trackFile.close();
              }
              sprintf(filename, "%s-LAPTIMES.csv", histDate);
              lapFile = sd.open(filename, FILE_READ);
              if (lapFile) {
                bestLapTime = 9999.00;
                while (lapFile.available()) {
                  csvReadUint16(&lapFile, &lapId, CSV_DELIM);
                  csvReadText(&lapFile, lapTime, sizeof(lapTime), CSV_DELIM);
                  csvReadFloat(&lapFile, &lapTimeSec, CSV_DELIM);
                  if (lapTimeSec < bestLapTime) {
                    bestLapTime = lapTimeSec;
                  }
                  BLUETOOTH_PORT.print("L");
                  BLUETOOTH_PORT.print(lapId);
                  BLUETOOTH_PORT.print(" > ");
                  BLUETOOTH_PORT.println(lapTime);
                }
                BLUETOOTH_PORT.print("Best : ");
                BLUETOOTH_PORT.println(bestLapTime);
                BLUETOOTH_PORT.println("----------------------------------");
                lapFile.close();
              }
            }
          }
          historyFile.close();
        }
        statId = 0;
        break;
      case 3:

        statId = 0;
        break;
      /**************************************************************
        #GetData > Print realtime data (like on the oled screen)
      **************************************************************/
      case 4:
        printData(fix_data, BLUETOOTH_PORT);
        statId = 0;
        break;
      case 5:

        break;
    }
  }

  /**************************************************************
    Get GPS frames through serial port (RX/TX)
    This is CRITICAL, data could be sent at any moment by the GPS so main loop should be executed in a minimum of time
    More information : https://github.com/SlashDevin/NeoGPS/blob/master/extras/doc/Troubleshooting.md#quiet-time-interval
  **************************************************************/
  if (gps.available(GPS_PORT)) {
    /**************************************************************
      Read last available GPS data
    **************************************************************/
    fix_data_prev = fix_data; // Memorize previous values before next GPS fix
    fix_data = gps.read(); // Get GPS data
    fixCount++;

    /**************************************************************
      Read all bike ECU values (RPM, Speed) + brake, throttle & gear states
    **************************************************************/
    elapsedTime = millis() - lastPinRead; // Used to have precise measures on RPM and SPEED
    lastPinRead = millis(); // Reset last pin read

    // Read RPM & SPEED counters
    REG_TCC0_CTRLBSET = TCC_CTRLBSET_CMD_READSYNC;  // Trigger a read synchronization on the COUNT register
    while (TCC0->SYNCBUSY.bit.CTRLB);               // Wait for the CTRLB register write synchronization
    while (TCC0->SYNCBUSY.bit.COUNT);               // Wait for the COUNT register read sychronization
    bikeSpeed = REG_TCC0_COUNT;                     // Read TCNT0 register (timer0 counter)
    REG_TCC0_COUNT = 0x0000;                        // Clear timer's COUNT value
    while (TCC0->SYNCBUSY.bit.COUNT);               // Wait for synchronization

    REG_TCC1_CTRLBSET = TCC_CTRLBSET_CMD_READSYNC;  // Trigger a read synchronization on the COUNT register
    while (TCC1->SYNCBUSY.bit.CTRLB);               // Wait for the CTRLB register write synchronization
    while (TCC1->SYNCBUSY.bit.COUNT);               // Wait for the COUNT register read sychronization
    bikeRpm = REG_TCC1_COUNT;                       // Read TCNT1 register (timer1 counter)
    REG_TCC1_COUNT = 0x0000;                        // Clear timer's COUNT value
    while (TCC1->SYNCBUSY.bit.COUNT);               // Wait for synchronization

    brake = digitalRead(brakePin); // Read "brakePin" (pin is plugged on the "+" of the stop light)
    throttle = min(((analogRead(throttlePin) / 10) * 1.23), 100); // Read voltage on "throttlePin"
    bikeRpm = bikeRpm * bikeRpmCoeff * 600 * 100 / elapsedTime / 22; // Ratio between pulse and rpm (22 > flywheel has 22 teeth ### 600 > with check every 100ms, RPM is by minute) ### (* 100 / elapsedTime) > if we read counter @101ms or 102ms values should be adjusted
    bikeSpeed = bikeSpeed * 100 / elapsedTime; // Speed ratio is 1 so no maths ### (* 100 / elapsedTime) > if we read counter @99ms, 101ms or 102ms values should be adjusted
    gearValue = analogRead(gearPin); // Read voltage on "gearPin" : every gear output a specific voltage (raw data measured on Daytona 675 : 314, 520, 660, 787, 888, 975, 1023 (N))
    if (gearValue <= 433) {
      gear = '1';
    }
    if (gearValue > 433 && gearValue <= 590) {
      gear = '2';
    }
    if (gearValue > 590 && gearValue <= 723) {
      gear = '3';
    }
    if (gearValue > 723 && gearValue <= 837) {
      gear = '4';
    }
    if (gearValue > 837 && gearValue <= 931) {
      gear = '5';
    }
    if (gearValue > 931 && gearValue <= 997) {
      gear = '6';
    }
    if (gearValue > 998) { // "N" = 1023
      if (gearNCheck > 3) { // We test 3 times to prevent displaying "N" between 2 gears
        gear = 'N';
      } else {
        gearNCheck++;
      }
    } else {
      gearNCheck = 0;
    }

    /**************************************************************
      Read 9-axis data
    **************************************************************/
    bno.getEvent(&event);

    /**************************************************************
      Read I2C analog values
    **************************************************************/
    readAdcValues(an1Value, an2Value, an3Value, an4Value, an5Value, an6Value, an7Value, an8Value);

    /**************************************************************
      Sync files on SDcard every 300 fixes (300x100ms = 30sec) to avoid dataloss on power failure
    **************************************************************/
    if (isRunning && (fixCount - lastSdSync >= 300)) {
      lastSdSync = fixCount;
      logFile.sync();
      lapFile.sync();
    }

    /**************************************************************
      One second loop (every 10 fixes > 10x100ms = 1sec), could be used for :
      - Displaying on OLED screen
      - Get values with a low refresh rate (temperature)
      - ...
    **************************************************************/
    if (fixCount - lastLCDupdate >= 10) {
      lastLCDupdate = fixCount;

      // Get other data (not time critical, so every 1 sec only)
      temperature = bno.getTemp(); // Temperature
      //bno.getCalibration(&cal_sys, &cal_gyro, &cal_accel, &cal_mag); // Calibration status

      if (isRunning) {
        // Blinking LED (user knows it's running), no other data is printed on LCD because it can be time consumming and we'll loose some GPS frames
        digitalWrite(ledPin, digitalRead(ledPin) ^ 1);

        /**************************************************************
          Print data on OLED screen when laptimer IS running - NOT TESTED
        **************************************************************/
        /*
          if (lapCounter > 0) { // Get first laptime at the end of the lap 1 (lapCounter = 1 / We start from the paddocks)
            OLED_PORT.setCursor(0, 4);
            TimeSubstract(fix_data_prev.dateTime, fix_data_prev.dateTime_cs, lastFlSec, lastFlCsec, lapSec, lapCsec);
            runMinutes = lapSec / 60;
            runSeconds = lapSec % 60;
            OLED_PORT.print(runMinutes);
            OLED_PORT.print(".");
            if (runSeconds < 10) OLED_PORT.print("0"); // Leading zeros (remember "timeCsec" is an integer !!)
            OLED_PORT.print(runSeconds);
            OLED_PORT.print(".");
            if (lapCsec < 10) OLED_PORT.print("0"); // Leading zeros (remember "timeCsec" is an integer !!)
            OLED_PORT.println(lapCsec);
          }
        */
      } else {
        /**************************************************************
          Print data on OLED screen when laptimer IS NOT running
        **************************************************************/
        OLED_PORT.setCursor(0, 0);
        printData(fix_data, OLED_PORT);
      }
    }

    /**************************************************************
      If laptimer is running :
      - Calculate distances, laptime with GPS data
      - Log data on SDcard
    **************************************************************/
    if (isRunning) {
      // Calculate total distance (for SeriousRacing)
      coordsDistance = gpsDistance(fix_data_prev.latitudeL(), fix_data_prev.longitudeL(), fix_data.latitudeL(), fix_data.longitudeL());
      totalDistance += coordsDistance;

      // Check if we pass the finishline (2x2 coordinates for finish line points + 2x2 coordinates for last position + position now)
      if (recordTrackData == true) {
        if (segIntersect(fix_data.latitudeL(), fix_data.longitudeL(), fix_data_prev.latitudeL(), fix_data_prev.longitudeL(), flineLat1, flineLon1, flineLat2, flineLon2, posCrossLat, posCrossLon) || (throttle == 100)) {
          // Calculate Time To Finish Line (from last know position by GPS just before crossing the finish line / format : sss.ms) ### tToFl = (distance between previous position and finish line (Ex : 0.00112km) / distance between previous position and actual position (Ex : 0.00254km)) * (time of actual fix - time of previous fix)
          //tToFl = (gpsDistance(fix_data_prev.latitudeL(), fix_data_prev.longitudeL(), posCrossLat, posCrossLon) / gpsDistance(fix_data.latitudeL(), fix_data.longitudeL(), fix_data_prev.latitudeL(), fix_data_prev.longitudeL())) * ((fix_data.dateTime.hours * 3600 + fix_data.dateTime.minutes * 60 + fix_data.dateTime.seconds + fix_data.dateTime_cs / 100.00) - (fix_data_prev.dateTime.hours * 3600 + fix_data_prev.dateTime.minutes * 60 + fix_data_prev.dateTime.seconds + fix_data_prev.dateTime_cs / 100.00));

          /*************************
            TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST
          *************************/
          tToFl = 0;


          // Add "Time to finish line (tToFl)" to the last known Epoch Time (fix_data_prev)
          TimeAdd(tToFl, fix_data_prev.dateTime, fix_data_prev.dateTime_cs, timeSec, timeCsec);

          // Calculate total laptime (substract previous finish laptime to actual laptime)
          if (lapCounter > 0) { // Get first laptime at the end of the lap 1 (lapCounter = 1 / We start from the paddocks)
            TimeSubstract(timeSec, timeCsec, lastFlSec, lastFlCsec, lapSec, lapCsec);
            runMinutes = lapSec / 60;
            runSeconds = lapSec % 60;
            lapFile.print(lapCounter);
            lapFile.print(";");
            lapFile.print(runMinutes);
            lapFile.print(":");
            if (runSeconds < 10) lapFile.print("0"); // Leading zeros (remember "runSeconds" is an integer !!)
            lapFile.print(runSeconds);
            lapFile.print(".");
            if (lapCsec < 10) lapFile.print("0"); // Leading zeros (remember "lapCsec" is an integer !!)
            lapFile.print(lapCsec);
            lapFile.print(";");
            lapFile.print(lapSec);
            lapFile.print(".");
            lapFile.println(lapCsec);

            OLED_PORT.print(lapCounter);
            OLED_PORT.print(" : ");
            OLED_PORT.print(runMinutes);
            OLED_PORT.print(":");
            if (runSeconds < 10) OLED_PORT.print("0"); // Leading zeros (remember "runSeconds" is an integer !!)
            OLED_PORT.print(runSeconds);
            OLED_PORT.print(".");
            if (lapCsec < 10) OLED_PORT.print("0"); // Leading zeros (remember "timeCsec" is an integer !!)
            OLED_PORT.print(lapCsec);
            OLED_PORT.print(";");
            OLED_PORT.print(lapSec);
            OLED_PORT.print(".");
            OLED_PORT.println(lapCsec);
          }

          // Store timestamp (sec+ms) at the finish line to calculate next lap time
          lastFlSec = timeSec;
          lastFlCsec = timeCsec;

          // Write the finish log line + inc lapCounter
          addFinishLog = true;
          lapCounter++;
        } else {
          addFinishLog = false;
        }
      }

      //write pins, IMU, GPS to SD card
      if (addFinishLog == true) {
        logFile.print(timeSec);
        logFile.print(".");
        if (timeCsec < 10) logFile.print("0"); // Leading zeros (remember "timeCsec" is an integer !!)
        logFile.print(timeCsec);
      } else {
        logFile.print(fix_data.dateTime);
        logFile.print(".");
        if (fix_data.dateTime_cs < 10) logFile.print("0"); // Leading zeros (remember "fix_data.dateTime_cs" is an integer !!)
        logFile.print(fix_data.dateTime_cs);
      }
      logFile.print(";");
      logFile.print(totalDistance, 3);
      logFile.print(";");
      logFile.print(lapCounter);
      logFile.print(";");
      logFile.print(brake);
      logFile.print(";");
      logFile.print(throttle);
      logFile.print(";");
      logFile.print(gear);
      logFile.print(";");
      logFile.print(bikeRpm);
      logFile.print(";");
      logFile.print(bikeSpeed);
      logFile.print(";");
      logFile.print(fix_data.speed_kph(), 0);
      logFile.print(";");
      logFile.print(fix_data.heading(), 1);
      logFile.print(";");
      logFile.print(event.orientation.y, 0);
      logFile.print(";");
      logFile.print(event.orientation.z, 0);
      logFile.print(";");
      logFile.print(temperature);
      logFile.print(";");
      if (addFinishLog == true) {
        logFile.print((posCrossLat / rescaleGPS), 9);
        logFile.print(";");
        logFile.println((posCrossLon / rescaleGPS), 9);
      } else {
        logFile.print(fix_data.latitude(), 9);
        logFile.print(";");
        logFile.println(fix_data.longitude(), 9);
      }
    }
  }
}
