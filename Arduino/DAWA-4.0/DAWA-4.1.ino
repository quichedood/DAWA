/*
   DAWA 4.0 (Arduino M0)
   Triumph motorbikes laptimer/datalogger
   Edouard PIGEON - 2017
*/

// Librairies
#include <SPI.h>

/*
   NeoGPS library
   https://github.com/SlashDevin/NeoGPS
*/
#include <NMEAGPS.h>
HardwareSerial & gps_port = Serial5;
#define DEBUG_PORT SERIAL_PORT_USBVIRTUAL // Debug to USB serial console
static NMEAGPS  gps;
static gps_fix  fix_data;
static gps_fix  fix_data_prev;

/*
   thomasfredericks/Bounce2 library
   https://github.com/thomasfredericks/Bounce2
*/
#include <Bounce2.h>
Bounce debouncer = Bounce();

/*
   Greiman/SdFat library
   https://github.com/greiman/SdFat
*/
#include <SdFat.h>

/*
   Modified dtostrf function defnition
   http://forum.arduino.cc/index.php?topic=368720.0
*/
#include <dtostrf.h>

/*
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
*/
#include <extEEPROM.h> // Arduino External EEPROM Library v3.0 (Official Github : https://github.com/JChristensen/extEEPROM)
extEEPROM eep(kbits_2, 1, 8); // I2C Address 0x50

/*
   Modification of the EEPROMAnything library that allows you to read and write any data structure or array to and from the external EEPROM
   https://forum.arduino.cc/index.php?topic=358648.0
*/
#include <EEPROMAnything2.h> // Enable object read/write on EEPROM

/*
   LCD library
   SPI ASCII OLED : https://github.com/greiman/SSD1306Ascii
*/
// SPI ASCII OLED libraries
#include "SSD1306Ascii.h" // OLED LCD based on SSD1306 chip, ASCII only no graphics
#include "SSD1306AsciiSpi.h"
SSD1306AsciiSpi oled; // 128x64 LCD OLED
#define OLED_DC 9
#define OLED_CS 13

/*
   9-axis Bosch BNO055 Adafruit library
   https://github.com/adafruit/Adafruit_BNO055
*/
#include <Adafruit_BNO055.h> // Adafruit 9-axis BNO_055 lib
#include <Adafruit_Sensor.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS_B); // USE_BNO055_ADDRESS_B = Adress 0x29 (COM3 pin set to 1)

/*
   Variable PINs I/O
*/
#define CSV_DELIM ';'
const float bikeRpmCoeff = 1.05; // Enable high RPM correction
const unsigned int maxTrackDistance = 5; // Autoselect nearer track (unit = km)
const uint8_t ledPin = 4; // D4
const uint8_t powerPin = A3; //A3
const uint8_t brakePin = 11; // D11
const uint8_t throttlePin = A0; // A0
const uint8_t gearPin = A1; // A1
const uint8_t sdCsPin = 7; // D7
const uint8_t oledResetPin = 8; // D8
const uint8_t timepulsePin = A2; //A2
const unsigned char ubxRate10Hz[] PROGMEM = {0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00};
const unsigned char ubxTimepulse[] PROGMEM = {0x06, 0x31, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29, 0x00, 0x00, 0x00};
//                                           |ID         |Lenght     |TP   |res  |res        |antCableD  |rfGrDelay  |freqPeriod             |freqPeriod lock        |Pulselenghtratio       |Pulselenghtratiolock   |UserConfigDelay        |Flags                 |
const unsigned char ubxPrtConf[] PROGMEM = {0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xd0, 0x08, 0x00, 0x00, 0x00, 0xc2, 0x01, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00}; // 115200bps
//                                         |ID         |Lenght     |Port |res  |TX Ready   |mode                   |baudrate               |inPrMask   |outPrMask  |flags      |res       |
// Disable specific NMEA sentences
const unsigned char ubxDisableGLL[] PROGMEM = {0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const unsigned char ubxDisableGSA[] PROGMEM = {0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const unsigned char ubxDisableGSV[] PROGMEM = {0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const unsigned char ubxEnableRMC[] PROGMEM = {0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
const unsigned char ubxDisableVTG[] PROGMEM = {0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const unsigned char ubxDisableZDA[] PROGMEM = {0x06, 0x01, 0x08, 0x00, 0xF0, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/*
   Vars
*/
unsigned long lastPinRead = 0;
unsigned long lastLCDupdate = 0;
unsigned long lastSdSync = 0;
unsigned long stopwatchStartTime = 0;
unsigned int elapsedTime = 0;
char trackName[16];
bool recordTrackData;
byte lapCounter = 0;
bool addFinishLog = 0;

// GPS
bool gpsDataReady = false;
float flineLat1, flineLon1, flineLat2, flineLon2 = 0;
float posCrossLat, posCrossLon;
float distanceKm;
float tToFl;

// Sensors
bool sensorsDataReady = false;
unsigned int bikeRpm;
unsigned int bikeSpeed;
unsigned int gearValue;
char gear = 'N';
byte gearNCheck = 0;
bool brake = false;
bool isRunning = false;
unsigned int throttle;

// SD Card
SdFat sd;
File logFile;
File trackFile;
char filename[20];

// 9-axis sensor
byte temperature; // Don't go out when freezing ;)
byte cal_sys, cal_gyro, cal_accel, cal_mag = 0;
adafruit_bno055_offsets_t calibrationData;
sensors_event_t event;
//byte *calibrationBuffer = (byte *) &calibrationData;

/*
   Triggered when initialization error (program stop and slow blinking led)
*/
void initerror() {
  delay(2000); // Pause 3 sec pour lecture infos LCD
  while (1) {
    digitalWrite(ledPin, digitalRead(ledPin) ^ 1); // Quick LED blink (there's an error)
    delay(200);
  }
}

/*
   Send UBX commands to UBLOX GPS
*/
void sendUBX(const unsigned char *progmemBytes, size_t len) {
  gps_port.write(0xB5); // SYNC1
  gps_port.write(0x62); // SYNC2

  uint8_t a = 0, b = 0;
  while (len-- > 0) {
    uint8_t c = pgm_read_byte(progmemBytes++);
    a += c;
    b += a;
    gps_port.write(c);
  }
  gps_port.write(a); // CHECKSUM A
  gps_port.write(b); // CHECKSUM B
}

/*
   Read CSV file
   https://github.com/greiman/SdFat/blob/master/examples/ReadCsv/ReadCsv.ino

   Read a file one field at a time.
   file - File to read.
   str - Character array for the field.
   size - Size of str array.
   delim - csv delimiter.
   return - negative value for failure.
            delimiter, '\n' or zero(EOF) for success.
*/
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

/*
   Calculate 2 line segments intersection
*/
bool segIntersect(float pos_now_lat, float pos_now_lon, float pos_prev_lat, float pos_prev_lon, float &trackLat1, float &trackLon1, float &trackLat2, float &trackLon2, float &pos_cross_lat, float &pos_cross_lon) {
  float denom, s_numer, t_numer, t, track_pos_x, track_pos_y, pos_x, pos_y, trackLon, trackLat;
  bool denomPositive;

  /*DEBUG_PORT.print("TR:");
    DEBUG_PORT.print(trackLat1, 8);
    DEBUG_PORT.print(";");
    DEBUG_PORT.print(trackLon1, 8);
    DEBUG_PORT.print(";");
    DEBUG_PORT.print(trackLat2, 8);
    DEBUG_PORT.print(";");
    DEBUG_PORT.println(trackLon2, 8);
    DEBUG_PORT.print("POS:");
    DEBUG_PORT.print(pos_now_lat, 8);
    DEBUG_PORT.print(";");
    DEBUG_PORT.print(pos_now_lon, 8);
    DEBUG_PORT.print(";");
    DEBUG_PORT.print(pos_prev_lat, 8);
    DEBUG_PORT.print(";");
    DEBUG_PORT.println(pos_prev_lon, 8);*/

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

/*
   Calculate distance between 2 GPS coords (unit = km, precision = meters) - haversine formula
*/
float gpsDistance(float lat1, float lon1, float lat2, float lon2) {
  float p = 0.017453292519943295; // (Pi / 180)
  float a = 0.5000 - cos((lat2 - lat1) * p) / 2 + cos(lat1 * p) * cos(lat2 * p) * (1 - cos((lon2 - lon1) * p)) / 2;
  return 12742 * asin(sqrt(a)); // 12742 = 2x earth radius
}

/*
   Initialisation (I/O, LCD, SD, DATE, 9-AXIS SENSOR, TIMERS/COUNTERS)
*/
void setup() {
  Wire.begin(); // I2C bus init

  // GPS Init
  gps_port.begin(115200); // Start the UART for the GPS device (you cannot block for more than "buffer size" * 11 / 9600 = 70ms)
  sendUBX(ubxRate10Hz, sizeof(ubxRate10Hz)); // Set refresh rate to 10Hz
  sendUBX(ubxTimepulse, sizeof(ubxTimepulse)); // Set timepulse output ON
  sendUBX(ubxPrtConf, sizeof(ubxPrtConf)); // Set UART speed to 115200bps (Warning : @9600bps > ~5sec delay on GPS data)
  sendUBX(ubxDisableGLL, sizeof(ubxDisableGLL));
  sendUBX(ubxDisableGSA, sizeof(ubxDisableGSA));
  sendUBX(ubxDisableGSV, sizeof(ubxDisableGSV));
  sendUBX(ubxEnableRMC, sizeof(ubxEnableRMC));
  sendUBX(ubxDisableVTG, sizeof(ubxDisableVTG));
  sendUBX(ubxDisableZDA, sizeof(ubxDisableZDA));

  // Init I/O pins
  pinMode(ledPin, OUTPUT); // Button light
  pinMode(powerPin, INPUT); // ON/OFF button
  pinMode(brakePin, INPUT); // Brake sensor (0v/12v)
  pinMode(throttlePin, INPUT); // Throttle sensor (analogic from 0v to 5v)
  pinMode(gearPin, INPUT); // Gear sensor (analogic from 0v to 5v)
  pinMode(oledResetPin, OUTPUT); // Reset on OLED screen
  pinMode(timepulsePin, INPUT); // NOT USED - Timepulse output from ublox GPS chip (set to 10Hz)

  // Debounce power button
  debouncer.attach(powerPin);
  debouncer.interval(2000); // interval in ms

  // Init + clear LCD
  digitalWrite(oledResetPin, HIGH); // Reset pin on OLED screen should be tied to +VCC
  oled.begin(&Adafruit128x64, OLED_CS, OLED_DC); oled.setFont(System5x7); // SPI or I2C
  //oled.begin(&Adafruit128x64, I2C_ADDRESS);oled.setFont(System5x7); // SPI or I2C

  delay(1000); // LCD init delay
  oled.clear();

  // Init EEPROM
  oled.setCursor(0, 0);
  oled.print("EEPROM:");
  if (eep.begin(twiClock400kHz) == 0) {
    oled.println("            OK");
  } else {
    oled.println("        FAILED");
    initerror();
  }

  // Init SD card
  oled.print("SD:");
  if (!sd.begin(sdCsPin, SD_SCK_MHZ(50))) {
    oled.println("            FAILED");
    initerror();
  }
  oled.println("                OK");

  // Init 9-axis sensor
  oled.print("9-AXIS:");
  if (!bno.begin()) {
    oled.println("        FAILED");
    initerror();
  }
  oled.println("            OK");
  bno.setExtCrystalUse(true); // Use external quartz for 9-axis sensor
  oled.println(">> PRESS TO CALIB. <<");
  digitalWrite(ledPin, HIGH); // We light up the led for the user (calibration is possible)
  delay(2000); // LCD user read time
  if (digitalRead(A3) == 1) { // If user press button, we start calibration
    oled.print("CALIBRATE:");
    while (!bno.isFullyCalibrated()) { // We wait until sensor fully calibrated !! Offsets are returned only if sensor is calibrated !!
      delay(200);
      bno.getCalibration(&cal_sys, &cal_gyro, &cal_accel, &cal_mag); // Get calibration state (from 0 to 3 for the 3 sensors + main sensor calibration)
      oled.setCursor(84, 4);
      oled.print(cal_sys);
      oled.print(";");
      oled.print(cal_gyro);
      oled.print(";");
      oled.print(cal_accel);
      oled.print(";");
      oled.print(cal_mag);
      delay(200);
    }
    bno.getSensorOffsets(calibrationData); // Sensor is calibrated, we save offsets to var (calibrationData)
    if (EEPROM_writeAnything(0, calibrationData) == sizeof(adafruit_bno055_offsets_t)) { // We save the 11 offsets to eeprom
      oled.println("  OK");
    } else {
      initerror();
    }
  } else { // Calibration not asked
    oled.print("LOAD CALIB.");
    if (EEPROM_readAnything(0, calibrationData) == sizeof(adafruit_bno055_offsets_t)) { // We try loading offsets from EEPROM
      bno.setSensorOffsets(calibrationData); // We set them in var
      oled.println("        OK");
    } else {
      oled.println("   NO DATA");
      initerror(); // Calibration not asked and no offset in EEPROM > can't go further !
    }
  }
  digitalWrite(ledPin, LOW); // Calibration is done, LED off

  // End Init
  oled.print("              READY !");
  delay(1000); // LCD user read time
  oled.clear();

  /*
      DEPRECATED / A 100ms interrupt based on GPS Ublox timepulse signal // Call <function_name> on each interruption
  */
  //attachInterrupt(digitalPinToInterrupt(timepulsePin), <function_name>, FALLING);

  /*
      2 counters (TCC0 & TCC1)
      for SPEED and RPM values

      m0 pinout : https://github.com/arduino/ArduinoCore-samd/blob/master/variants/arduino_mzero/variant.cpp
      Big thanks to : https://forum.arduino.cc/index.php?topic=396804.45
  */
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

// Main loop
void loop() {
  // Update the Bounce instance :
  debouncer.update();

  // ON/OFF
  if (debouncer.rose()) { // long press on power button
    if (isRunning) { // If stopwatch is running ...
      isRunning = false; // ... we stop recording
      digitalWrite(ledPin, 0); // LED off
      logFile.close(); // Close file on SDcard
      oled.clear();
    } else {
      oled.clear();
      oled.println("Running ...");

      // Select nearer track
      recordTrackData = false;
      trackFile = sd.open("TRACKS.csv", FILE_READ);
      if (trackFile) {
        while (trackFile.available()) {
          csvReadText(&trackFile, trackName, sizeof(trackName), CSV_DELIM); // One line per track : "CAROLE;48.979993;2.522435;48.980023;2.522633" (<trackname>;<startline_a_lat>;<startline_a_lon>;<startline_b_lat>;<startline_b_lon>)
          csvReadFloat(&trackFile, &flineLat1, CSV_DELIM);                  // Points A & B should be at the left and at the right of the finishline (a few meters)
          csvReadFloat(&trackFile, &flineLon1, CSV_DELIM);
          csvReadFloat(&trackFile, &flineLat2, CSV_DELIM);
          csvReadFloat(&trackFile, &flineLon2, CSV_DELIM);

          distanceKm = gpsDistance(fix_data.latitude(), fix_data.longitude(), flineLat1, flineLon1); // Get the nearer track from us
          if (distanceKm <= maxTrackDistance) {
            recordTrackData = true;
            oled.print(trackName);
            oled.print(" (");
            oled.print(distanceKm, 1);
            oled.println("km)");
            break;
          }
        }
        trackFile.close();
      }
      if (recordTrackData == false) {
        oled.println("No tracks file !");
      }

      // Create datafile
      sprintf(filename, "%02u%02u%02u-%02u%02u%02u.txt", fix_data.dateTime.year, fix_data.dateTime.month, fix_data.dateTime.date, fix_data.dateTime.hours, fix_data.dateTime.minutes, fix_data.dateTime.seconds);
      if (logFile.open(filename, O_CREAT | O_WRITE | O_EXCL)) {
        logFile.println(F("time;lap;brake;throttle;gear;rpm;kph;kphgps;heading;roll;pitch;temp;lat;lon"));
      } else {
        initerror();
      }

      // Define stopwatch start time
      isRunning = true; // ... we start recording
      lapCounter = 0;
      stopwatchStartTime = millis();
    }
  }

  // Get GPS frames through serial port (this is CRITICAL, data could be sent at any moment by the GPS so main loop should be executed in a minimum of time)
  if (gps.available(gps_port)) { // && gpsDataReady == false ?
    fix_data_prev = fix_data; // Memorize previous values before next GPS fix
    fix_data = gps.read();
    gpsDataReady = true;
  }

  // Every ~100ms (10Hz), we get all values (RPM, Speed, brake, gear, 9-axis data)
  elapsedTime = millis() - lastPinRead;
  if (elapsedTime >= 100) {
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
    gearValue = analogRead(gearPin); // Read voltage on "gearPin" : every gear output a specific voltage (raw data measured on Daytona 675 : 314, 520, 660, 787, 888, 975, 1023 (N))
    bikeRpm = bikeRpm * bikeRpmCoeff * 600 * 100 / elapsedTime / 22; // Ratio between pulse and rpm (22 > flywheel has 22 teeth ### 600 > with check every 100ms, RPM is by minute) ### (* 100 / elapsedTime) > if we read counter @101ms or 102ms values should be adjusted
    bikeSpeed = bikeSpeed * 100 / elapsedTime; // Speed ratio is 1 so no maths ### (* 100 / elapsedTime) > if we read counter @101ms or 102ms values should be adjusted

    // Get 9-axis sensor data
    bno.getEvent(&event);

    // Gear selected
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
    sensorsDataReady = true;
  }

  // Sync file on SDcard every 30sec to avoid dataloss on power failure
  if (isRunning && (millis() - lastSdSync > 30000)) {
    lastSdSync = millis();
    logFile.sync();
  }

  /*
    Displaying on LCD
    Everything is optional
  */
  if (millis() - lastLCDupdate > 1000) {
    // Get other data (not time critical, so every 1 sec)
    temperature = bno.getTemp(); // Temperature
    //bno.getCalibration(&cal_sys, &cal_gyro, &cal_accel, &cal_mag); // Calibration status

    lastLCDupdate = millis();
    if (isRunning) {
      // Blinking LED (user knows it's running), no other data is printed on LCD because it's time consumming and we'll loose some GPS frame
      digitalWrite(ledPin, digitalRead(ledPin) ^ 1);
    } else {
      if (fix_data.valid.date && fix_data.valid.time) {
        oled.setCursor(0, 0);
        if (fix_data.dateTime.date < 10) oled.print("0"); // Leading zeros
        oled.print(fix_data.dateTime.date);
        oled.print("/");
        if (fix_data.dateTime.month < 10) oled.print("0"); // Leading zeros
        oled.print(fix_data.dateTime.month);
        oled.print("/");
        if (fix_data.dateTime.year < 10) oled.print("0"); // Leading zeros
        oled.print(fix_data.dateTime.year);
        oled.print(" ");
        if (fix_data.dateTime.hours < 10) oled.print("0"); // Leading zeros
        oled.print(fix_data.dateTime.hours);
        oled.print(":");
        if (fix_data.dateTime.minutes < 10) oled.print("0"); // Leading zeros
        oled.print(fix_data.dateTime.minutes);
        oled.print(":");
        if (fix_data.dateTime.seconds < 10) oled.print("0"); // Leading zeros
        oled.println(fix_data.dateTime.seconds);
      } else {
        oled.println("No GPS signal");
      }
      oled.print("RPM:       ");
      oled.println(bikeRpm);
      oled.print("SPEED:     ");
      oled.println(bikeSpeed);
      oled.print("GEAR:      ");
      oled.println(gear);
      oled.print("BRAKE:     ");
      oled.println(brake);
      oled.print("THROTTLE:  ");
      oled.println(throttle);
      //oled.print("LAP:     ");
      //oled.println(lapCounter);

      /*49.004837036,2.239411116
        49.004840851,2.239416122
        49.004866,2.239351
        49.004815,2.239474
        if (segIntersect(49.004837036,2.239411116, 49.004840851,2.239416122, 49.004866, 2.239351, 49.004815,2.239474, posCrossLat, posCrossLon)) {
        oled.println("CROSS");

        } else {
        oled.println("NOT CROSS");
        }*/

      oled.print("TEMP:      ");
      oled.println(temperature);
      oled.print("GPS:");
      oled.print(fix_data.dateTime);
      oled.print(".");
      oled.print(fix_data.dateTime_cs);
    }
  }

  if (isRunning) {
    if (gpsDataReady && sensorsDataReady) {
      // Check if we pass the finishline (2x2 coordinates for finish line points + 2x2 coordinates for last position + position now)
      if (segIntersect(fix_data.latitude(), fix_data.longitude(), fix_data_prev.latitude(), fix_data_prev.longitude(), flineLat1, flineLon1, flineLat2, flineLon2, posCrossLat, posCrossLon)) {
        addFinishLog = true;
        lapCounter++;

        // Time to finish line (from last know position brefore crossing the finish line) ### tToFl = (distance between previous position and finish line (Ex : 0.00112km) / distance between previous position and actual position (Ex : 0.00254km)) * (time of actual fix - time of previous fix)
        tToFl = (gpsDistance(fix_data_prev.latitude(), fix_data_prev.longitude(), posCrossLat, posCrossLon) / gpsDistance(fix_data.latitude(), fix_data.longitude(), fix_data_prev.latitude(), fix_data_prev.longitude())) * ((fix_data.dateTime.hours * 3600 + fix_data.dateTime.minutes * 60 + fix_data.dateTime.seconds + fix_data.dateTime_cs / 100.00) - (fix_data_prev.dateTime.hours * 3600 + fix_data_prev.dateTime.minutes * 60 + fix_data_prev.dateTime.seconds + fix_data_prev.dateTime_cs / 100.00));
      } else {
        addFinishLog = false;
      }

      //write pins, IMU, GPS to SD card
      DEBUG_PORT.print(fix_data_prev.dateTime);
      DEBUG_PORT.print(".");
      DEBUG_PORT.print(fix_data.dateTime_cs);
      DEBUG_PORT.print(";");
      DEBUG_PORT.print(fix_data.latitudeL());
      DEBUG_PORT.print(";");
      DEBUG_PORT.print(fix_data.longitudeL());
      DEBUG_PORT.print(";");
      DEBUG_PORT.print(fix_data.latitude(), 10);
      DEBUG_PORT.print(";");
      DEBUG_PORT.println(fix_data.longitude(), 10);

      if (addFinishLog == true) {
        logFile.print(fix_data_prev.dateTime + (int)(tToFl + (fix_data_prev.dateTime_cs / 100.00))); // Get full seconds of previous GPS fix + full seconds of : time to the finish line & hundredths of previous fix
        logFile.print(F("."));
        logFile.print((int)(((tToFl + (fix_data_prev.dateTime_cs / 100.00)) - (int)(tToFl + (fix_data_prev.dateTime_cs / 100.00))) * 100)); // Get hundredths left (integrer)
      } else {
        logFile.print(fix_data.dateTime);
        logFile.print(F("."));
        logFile.print(fix_data.dateTime_cs);
      }
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
        logFile.print(posCrossLat, 9);
        logFile.print(";");
        logFile.println(posCrossLon, 9);
      } else {
        logFile.print(fix_data.latitude(), 9);
        logFile.print(";");
        logFile.println(fix_data.longitude(), 9);
      }

      gpsDataReady = false;
      sensorsDataReady = false;
    }
  }
}
