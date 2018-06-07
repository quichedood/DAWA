/**************************************************************
  DAWA 5.0 (Arduino M0)
  Triumph motorbikes laptimer/datalogger
  Edouard PIGEON - 2017
**************************************************************/

/**************************************************************
  TODO
  Terminer/vérifier log fichiers
  Angle inclinaison
  Freeze I2C : I2C@100kHz + shielded cables + Pullup R @ 4.7k (10k)
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
  MLX90614 Infrared temperature sensor
  https://github.com/jfitter/MLX90614
**************************************************************/
#include <MLX90614.h>

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

  EEPROM addresses used :
  0-21 BNO Calibration offsets (11x uint16_t)
  30 ADC disabled bits (1x uint8_t)
  31-32 Throttle max value (1x int16_t)
  33-46 Gear calibration data (7x int16_t)
  50-55 MLX I2C Address (x uint8_t ... 6 max)
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
  Constants definition
**************************************************************/
// I/O Pins
const uint8_t ledPin = 4; // D4
const uint8_t powerPin = A3; //A3
const uint8_t brakePin = 11; // D11
const uint8_t throttlePin = A0; // A0
const uint8_t gearPin = A1; // A1
const uint8_t sdCsPin = 5; // D5 (was D7 in previous versions)
const uint8_t oledResetPin = 8; // D8

// Others ...
const char csvDelim = ';'; // CSV file delimiter
const float bikeRpmCoeff = 1.05; // Enable high RPM correction
const uint8_t maxTrackDistance = 5; // Autoselect nearest track (unit = km)
const uint8_t gearOffset = 100; // Each gear has a corresponding value, this value define the interval (value - gearOffset < measure < value + gearOffset)
const uint8_t maxMlx = 4; // Max MLX chips that could be declared (could be more but need to add object instances L.223, just before setup())
const uint8_t mlxEepAddr = 0x0E; // Internal EEPROM address on MLX chips
const uint8_t firstMlxAddress = 0x1A;

// GPS & Timing
const float rescaleGPS = 10000000.0; // We use "long" for GPS coordinates to keep precision ("float" on Arduino have only 6 decimal digits of precision) ### https://gis.stackexchange.com/questions/8650/measuring-accuracy-of-latitude-and-longitude

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
  Vars definition
**************************************************************/

// GPS & Timing
bool recordTrackData, addFinishLog, isRunning = false;
char trackName[16];
uint8_t lapCounter = 0;
int16_t runMinutes, runSeconds;
uint16_t trackId, histTrackId, lapId;
int32_t timeCsec, timeSec, lastFlSec, lastFlCsec, lapSec, lapCsec, posCrossLat, posCrossLon, flineLat1, flineLon1, flineLat2, flineLon2; // Finish line GPS coordinates
uint32_t lastPinRead = 0, lastLCDupdate = 0, lastSdSync = 0, fixCount = 0, elapsedTime = 0;
float coordsDistance, totalDistance, tToFl; // Time To Finish Line

// Sensors
bool brake;
char gear;
uint8_t gearNCheck = 0;
uint16_t gearValue, throttle, gearValues[7], throttleMax;
uint32_t bikeRpm, bikeSpeed;

// SD Card
SdFat sd;
File logFile; // One line every 100ms with all detailed data
File lapFile; // One line per lap with laptime
File trackFile; // One line per track with GPS coordinates of finish line
File historyFile; // History of all sessions
char filename[32];

// 9-axis sensor
int8_t temperature;
uint8_t calSys, calGyro, calAccel, calMag;
adafruit_bno055_offsets_t calibrationData;
sensors_event_t event;

// Bluetooth
char c, cmdBuffer[16];

// Analog to Digital converter (ADC)
uint8_t anaDisabledBits = 0, tmpComp, bitShift;
int16_t anaValues[8];

// Infrared temp sensor
uint8_t mlxAddresses[maxMlx];
double mlxValues[maxMlx];
MLX90614 mlx[maxMlx] = {MLX90614(firstMlxAddress), MLX90614(firstMlxAddress + 1), MLX90614(firstMlxAddress + 2), MLX90614(firstMlxAddress + 3)};

/**************************************************************
  Initialisation (I/O, BLUETOOTH, LCD, SD, DATE, 9-AXIS SENSOR, TIMERS/COUNTERS)
**************************************************************/
void setup() {
  // Power on led, power off only when system started and valid GPS fix
  digitalWrite(ledPin, true);

  // I2C bus init
  Wire.begin();

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
  OLED_PORT.print(F("BLUETOOTH:"));
  BLUETOOTH_PORT.begin(9600);
  pinPeripheral(6, PIO_SERCOM); // Pin D6 for RX
  pinPeripheral(7, PIO_SERCOM); // Pin D7 for TX
  // ADD A TRUE BLUETOOTH TEST HERE !!
  OLED_PORT.println(F("         OK"));
  BLUETOOTH_PORT.println(F("D.A.W.A. Initializing ..."));
  BLUETOOTH_PORT.println(F("BLUETOOTH : OK"));

  // GPS Init (Serial5 - Default on Arduino m0)
  OLED_PORT.print(F("GPS:"));
  BLUETOOTH_PORT.print(F("GPS : "));
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
  OLED_PORT.println(F("               OK"));
  BLUETOOTH_PORT.println(F("OK"));

  // EEPROM Init (I2C)
  OLED_PORT.print(F("EEPROM:"));
  BLUETOOTH_PORT.print(F("EEPROM : "));
  if (!eep.begin(twiClock100kHz) == 0) {
    OLED_PORT.println(F("        FAILED"));
    BLUETOOTH_PORT.println(F("FAILED"));
    initError();
  } else {
    OLED_PORT.println(F("            OK"));
    BLUETOOTH_PORT.println(F("OK"));
  }
  
  // Read THROTTLE calibration data
  EEPROM_readAnything(31, throttleMax) == sizeof(throttleMax);

  // Read GEAR calibration data
  EEPROM_readAnything(33, gearValues) == sizeof(gearValues);

  // Read ADC disabled inputs
  EEPROM_readAnything(30, anaDisabledBits) == sizeof(anaDisabledBits);

  // Read saved MLX I2C Address (infrared temp sensors)
  EEPROM_readAnything(50, mlxAddresses) == sizeof(mlxAddresses);
  for (uint8_t i = 0; i < maxMlx; i++) {
    if (mlxAddresses[i] != 0x00) {
      mlx[i].begin();
    }
  }

  // ADC Init (I2C Address : 0x1D)
  OLED_PORT.print(F("ADC:"));
  BLUETOOTH_PORT.print(F("ADC : "));
  if (!initADC() == 0) {
    OLED_PORT.println(F("           FAILED"));
    BLUETOOTH_PORT.println(F("FAILED"));
    initError();
  } else {
    OLED_PORT.println(F("               OK"));
    BLUETOOTH_PORT.println(F("OK"));
  }

  // SD card Init (SPI)
  OLED_PORT.print(F("SD:"));
  BLUETOOTH_PORT.print(F("SD : "));
  if (!sd.begin(sdCsPin, SD_SCK_MHZ(50))) {
    OLED_PORT.println(F("            FAILED"));
    BLUETOOTH_PORT.println(F("FAILED"));
    initError();
  } else {
    OLED_PORT.println(F("                OK"));
    BLUETOOTH_PORT.println(F("OK"));
  }

  // 9-axis sensor Init (I2C)
  OLED_PORT.print(F("9-AXIS:"));
  BLUETOOTH_PORT.print(F("9-AXIS : "));
  if (!bno.begin()) {
    OLED_PORT.println(F("        FAILED"));
    BLUETOOTH_PORT.println(F("FAILED"));
    initError();
  } else {
    OLED_PORT.println(F("            OK"));
    BLUETOOTH_PORT.println(F("OK"));
    bno.setExtCrystalUse(true); // Use external quartz for 9-axis sensor
    delay(500); // Delay, wait for BNO initialization before loading offsets
    EEPROM_readAnything(0, calibrationData) == sizeof(adafruit_bno055_offsets_t); // We try loading offsets from EEPROM
    bno.setSensorOffsets(calibrationData); // We set them in var
  }

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

  // End Init
  OLED_PORT.print(F("              READY !"));
  BLUETOOTH_PORT.println(F("READY !"));
  delay(1000);
  OLED_PORT.clear();
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
      /**************************************************************
        ******************* STOP RECORDING HERE *******************
      **************************************************************/
      isRunning = false; // ... we stop recording
      digitalWrite(ledPin, false); // LED off
      logFile.close(); // Close file on SDcard
      lapFile.close(); // Close file on SDcard
      OLED_PORT.clear();
      BLUETOOTH_PORT.println(F("[Session stopped !]"));
    } else { // If laptimer is NOT running ...
      if (fix_data.valid.location) { // We need GPS fix before starting
        /**************************************************************
          ******************* START RECORDING HERE *******************
        **************************************************************/
        OLED_PORT.clear();
        OLED_PORT.println(F("Running ..."));
        BLUETOOTH_PORT.println(F("[Start new session !]"));

        /**************************************************************
          Select nearest track from the file "TRACKS.csv" on sdcard
        **************************************************************/
        recordTrackData = false;
        trackFile = sd.open("TRACKS.csv", FILE_READ);
        if (trackFile) {
          while (trackFile.available()) {
            csvReadUint16(&trackFile, &trackId, csvDelim);
            csvReadText(&trackFile, trackName, sizeof(trackName), csvDelim); // One line per track : "1;CAROLE;489799930;25224350;489800230;25226330" (<trackname>;<startline_a_lat>;<startline_a_lon>;<startline_b_lat>;<startline_b_lon>)
            csvReadInt32(&trackFile, &flineLat1, csvDelim);                  // Points A & B should be at the left and at the right of the finishline (a few meters)
            csvReadInt32(&trackFile, &flineLon1, csvDelim);
            csvReadInt32(&trackFile, &flineLat2, csvDelim);
            csvReadInt32(&trackFile, &flineLon2, csvDelim);

            // Calculate distance between 2 GPS coordinates
            coordsDistance = gpsDistance(fix_data.latitudeL(), fix_data.longitudeL(), flineLat1, flineLon1) / 1000;

            // If you are on a known track, then we select it
            if (coordsDistance <= maxTrackDistance) {
              recordTrackData = true;
              OLED_PORT.print(trackName);
              OLED_PORT.print(F(" ("));
              OLED_PORT.print(coordsDistance, 1);
              OLED_PORT.println(F("km)"));
              BLUETOOTH_PORT.print(F("Track selected : "));
              BLUETOOTH_PORT.print(trackName);
              BLUETOOTH_PORT.print(F(" ("));
              BLUETOOTH_PORT.print(coordsDistance, 1);
              BLUETOOTH_PORT.println(F("km)"));
              break; // Break here so last read values are the good ones !
            }
          }
          trackFile.close();
        }
        if (recordTrackData == false) {
          OLED_PORT.println(F("No track file !"));
          BLUETOOTH_PORT.println(F("No track file !"));
        }

        /**************************************************************
          Create new datafile : history file (append)
        **************************************************************/
        if (recordTrackData == true) { // No track = no history !
          sprintf(filename, "HISTORY.csv");
          if (historyFile.open(filename, O_CREAT | O_APPEND | O_WRITE)) {
            historyFile.print(fix_data.dateTime);
            historyFile.print(F(";"));
            historyFile.print(fix_data.dateTime.year);
            if (fix_data.dateTime.month < 10) historyFile.print(F("0")); // Leading zeros
            historyFile.print(fix_data.dateTime.month);
            if (fix_data.dateTime.date < 10) historyFile.print(F("0")); // Leading zeros
            historyFile.print(fix_data.dateTime.date);
            historyFile.print(F("-"));
            if (fix_data.dateTime.hours < 10) historyFile.print(F("0")); // Leading zeros
            historyFile.print(fix_data.dateTime.hours);
            if (fix_data.dateTime.minutes < 10) historyFile.print(F("0")); // Leading zeros
            historyFile.print(fix_data.dateTime.minutes);
            if (fix_data.dateTime.seconds < 10) historyFile.print(F("0")); // Leading zeros
            historyFile.print(fix_data.dateTime.seconds);
            historyFile.print(F(";"));
            historyFile.println(trackId);
            historyFile.close(); // Close file on SDcard
            BLUETOOTH_PORT.print(F("Append file : "));
            BLUETOOTH_PORT.println(filename);
          } else {
            initError();
          }
        }

        /**************************************************************
          Create new datafile : log file (create new)
        **************************************************************/
        sprintf(filename, "%02u%02u%02u-%02u%02u%02u.csv", fix_data.dateTime.year, fix_data.dateTime.month, fix_data.dateTime.date, fix_data.dateTime.hours, fix_data.dateTime.minutes, fix_data.dateTime.seconds);
        if (logFile.open(filename, O_CREAT | O_WRITE | O_EXCL)) {
          logFile.print(F("Time;Distance;Lap;Brake;"));
          bitShift = B00000001;
          for (uint8_t i = 0; i < 8; i++) { // We parse the 8 analog values
            tmpComp = bitShift & anaDisabledBits;
            if (tmpComp != bitShift) { // If analog is enabled ...
              if (i == 6) {
                logFile.print(F("Throttle;"));
              } else if (i == 7) {
                logFile.print(F("Gear;"));
              } else {
                logFile.print(F("An"));
                logFile.print(i + 1);
                logFile.print(F(";"));
              }
            }
            bitShift = bitShift << 1;
          }
          logFile.print(F("RPM;KPH;KPHGPS;Heading;Roll;Pitch;Temp;"));
          for (uint8_t i = 0; i < maxMlx; i++) {
            if (mlxAddresses[i] != 0x00) {
              logFile.print(F("IRTemp"));
              logFile.print(i);
              logFile.print(F(";"));
            }
          }
          logFile.println(F("Latitude;Longitude"));
          BLUETOOTH_PORT.print(F("Create new file : "));
          BLUETOOTH_PORT.println(filename);
        } else {
          initError();
        }

        /**************************************************************
          Create new datafile : laptime file (create new)
        **************************************************************/
        sprintf(filename, "%02u%02u%02u-%02u%02u%02u-LAPTIMES.csv", fix_data.dateTime.year, fix_data.dateTime.month, fix_data.dateTime.date, fix_data.dateTime.hours, fix_data.dateTime.minutes, fix_data.dateTime.seconds);
        if (lapFile.open(filename, O_CREAT | O_WRITE | O_EXCL)) {
          BLUETOOTH_PORT.print(F("Create new file : "));
          BLUETOOTH_PORT.println(filename);
        } else {
          initError();
        }

        /**************************************************************
          Init some vars
        **************************************************************/
        isRunning = true; // ... we start recording
        lapCounter = 0; // Lap 0 (we start from paddocks)
        totalDistance = 0;
        addFinishLog = false;
      }
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
      Read ECU values (RPM, SPEED)
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

    bikeRpm = bikeRpm * bikeRpmCoeff * 600 * 100 / elapsedTime / 22; // Ratio between pulse and rpm (22 > flywheel has 22 teeth ### 600 > with check every 100ms, RPM is by minute) ### (* 100 / elapsedTime) > if we read counter @101ms or 102ms values should be adjusted
    bikeSpeed = bikeSpeed * 100 / elapsedTime; // Speed ratio is 1 so no maths ### (* 100 / elapsedTime) > if we read counter @99ms, 101ms or 102ms values should be adjusted

    /**************************************************************
      Read BRAKE state
    **************************************************************/
    brake = digitalRead(brakePin); // Read "brakePin" (pin is plugged on the "+" of the stop light)

    /**************************************************************
      Read THROTTLE state (based on last ADC values, see next)
    **************************************************************/
    throttle = constrain(map(anaValues[6], 0, throttleMax, 0, 100), 0, 100);

    /**************************************************************
      Read GEAR state (based on last ADC values, see next)
    **************************************************************/
    anaValues[7] = constrain(anaValues[7], 0, 4096);
    if (anaValues[7] <= gearValues[1] + gearOffset) {
      gear = '1';
    }
    if (anaValues[7] > gearValues[2] - gearOffset && anaValues[7] <= gearValues[2] + gearOffset) {
      gear = '2';
    }
    if (anaValues[7] > gearValues[3] - gearOffset && anaValues[7] <= gearValues[3] + gearOffset) {
      gear = '3';
    }
    if (anaValues[7] > gearValues[4] - gearOffset && anaValues[7] <= gearValues[4] + gearOffset) {
      gear = '4';
    }
    if (anaValues[7] > gearValues[5] - gearOffset && anaValues[7] <= gearValues[5] + gearOffset) {
      gear = '5';
    }
    if (anaValues[7] > gearValues[6] - gearOffset && anaValues[7] <= gearValues[6] + gearOffset) {
      gear = '6';
    }
    if (anaValues[7] > gearValues[0] - gearOffset && anaValues[7] <= gearValues[0] + gearOffset) {
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
    readAdcValues(anaValues); // Takes time to read all values (one voltage conversion = 12.2ms !)

    /**************************************************************
      Sync files on SDcard every 300 fixes (300x100ms = 30sec) to avoid dataloss on power failure
    **************************************************************/
    if (isRunning && (fixCount - lastSdSync >= 300)) {
      lastSdSync = fixCount;
      logFile.sync();
      lapFile.sync();
    }

    /**************************************************************
      1 second loop (every 10 fixes > 10x100ms = 1sec), could be used for :
      - Displaying on OLED screen
      - Get values with a low refresh rate like ambiant temperature
      - ...
    **************************************************************/
    if (fixCount - lastLCDupdate >= 10) {
      lastLCDupdate = fixCount;

      /**************************************************************
        Read and store MLX temperature in array
      **************************************************************/
      for (uint8_t i = 0; i < maxMlx; i++) {
        if (mlxAddresses[i] != 0x00) {
          mlxValues[i] = mlx[i].readTemp(MLX90614::MLX90614_SRC01, MLX90614::MLX90614_TC);
        }
      }

      /**************************************************************
        Read ambiant temperature
      **************************************************************/
      temperature = bno.getTemp();

      /**************************************************************
        Read BNO calibration state
      **************************************************************/
      bno.getCalibration(&calSys, &calGyro, &calAccel, &calMag);

      if (isRunning) {
        /**************************************************************
          Blinking LED (user knows it's running)
        **************************************************************/
        digitalWrite(ledPin, digitalRead(ledPin) ^ 1);

        /**************************************************************
          Print data on OLED screen when laptimer IS running - NOT TESTED
        **************************************************************/
        // No other data is printed on LCD because it can be time consumming and we'll loose some GPS frames        
      } else {
        /**************************************************************
          Power off led as soon as we get a valid gps signal (ready to go)
        **************************************************************/
        if (fix_data.valid.location) { // We need GPS fix before starting
          digitalWrite(ledPin, false);
        }

        /**************************************************************
          Print data on OLED screen when laptimer IS NOT running
        **************************************************************/
        OLED_PORT.setCursor(0, 0);
        //printData1(fix_data, OLED_PORT); // Select one of the 2 options to print different data on OLED screen
        printData2(fix_data, OLED_PORT);
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
        if (segIntersect(fix_data.latitudeL(), fix_data.longitudeL(), fix_data_prev.latitudeL(), fix_data_prev.longitudeL(), flineLat1, flineLon1, flineLat2, flineLon2, posCrossLat, posCrossLon)) { // *********** TEST ONLY *********** Add >  || throttle == 100
          // Calculate Time To Finish Line (from last know position by GPS just before crossing the finish line / format : sss.ms) ### tToFl = (distance between previous position and finish line (Ex : 0.00112km) / distance between previous position and actual position (Ex : 0.00254km)) * (time of actual fix - time of previous fix)
          tToFl = (gpsDistance(fix_data_prev.latitudeL(), fix_data_prev.longitudeL(), posCrossLat, posCrossLon) / gpsDistance(fix_data.latitudeL(), fix_data.longitudeL(), fix_data_prev.latitudeL(), fix_data_prev.longitudeL())) * ((fix_data.dateTime.hours * 3600 + fix_data.dateTime.minutes * 60 + fix_data.dateTime.seconds + fix_data.dateTime_cs / 100.00) - (fix_data_prev.dateTime.hours * 3600 + fix_data_prev.dateTime.minutes * 60 + fix_data_prev.dateTime.seconds + fix_data_prev.dateTime_cs / 100.00));
          //tToFl = random(1, 100) / 100.00; // *********** TEST ONLY ***********

          // Add "Time to finish line (tToFl)" to the last known Epoch Time (fix_data_prev)
          timeAdd(tToFl, fix_data_prev.dateTime, fix_data_prev.dateTime_cs, timeSec, timeCsec);

          /*BLUETOOTH_PORT.println(F("TEST :");
            BLUETOOTH_PORT.println(tToFl);
            BLUETOOTH_PORT.println(fix_data_prev.dateTime);
            BLUETOOTH_PORT.println(fix_data_prev.dateTime_cs);
            BLUETOOTH_PORT.println(timeSec);
            BLUETOOTH_PORT.println(timeCsec);*/


          // Calculate total laptime (substract previous finish laptime to actual laptime)
          if (lapCounter > 0) { // Get first laptime at the end of the lap 1 (lapCounter = 1 / We start from the paddocks)
            timeSubstract(timeSec, timeCsec, lastFlSec, lastFlCsec, lapSec, lapCsec);

            /*BLUETOOTH_PORT.println(lastFlSec);
              BLUETOOTH_PORT.println(lastFlCsec);
              BLUETOOTH_PORT.println(lapSec);
              BLUETOOTH_PORT.println(lapCsec);*/

            runMinutes = lapSec / 60;
            runSeconds = lapSec % 60;

            lapFile.print(lapCounter);
            lapFile.print(F(";"));
            lapFile.print(runMinutes); // Store laptime mm:sss:ms (human readable)
            lapFile.print(F(":"));
            if (runSeconds < 10) lapFile.print(F("0")); // Leading zeros (remember "runSeconds" is an integer !!)
            lapFile.print(runSeconds);
            lapFile.print(F(":"));
            if (lapCsec < 10) lapFile.print(F("0")); // Leading zeros (remember "lapCsec" is an integer !!)
            lapFile.print(lapCsec);
            lapFile.print(F(";")); // Store laptime sss.ms (enable float comparaison for best lap or other calculations)
            lapFile.print(lapSec);
            lapFile.print(F("."));
            lapFile.println(lapCsec);

            OLED_PORT.print(lapCounter);
            OLED_PORT.print(F(" : "));
            OLED_PORT.print(runMinutes);
            OLED_PORT.print(F(":"));
            if (runSeconds < 10) OLED_PORT.print(F("0")); // Leading zeros (remember "runSeconds" is an integer !!)
            OLED_PORT.print(runSeconds);
            OLED_PORT.print(F("."));
            if (lapCsec < 10) OLED_PORT.print(F("0")); // Leading zeros (remember "timeCsec" is an integer !!)
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

      /**************************************************************
        Write all data to file on SD card (IMU, GPS, Throttle, gear, rpm, temperature sensors)
      **************************************************************/
      if (addFinishLog == true) {
        logFile.print(timeSec);
        logFile.print(F("."));
        if (timeCsec < 10) logFile.print(F("0")); // Leading zeros (remember "timeCsec" is an integer !!)
        logFile.print(timeCsec);
      } else {
        logFile.print(fix_data.dateTime);
        logFile.print(F("."));
        if (fix_data.dateTime_cs < 10) logFile.print(F("0")); // Leading zeros (remember "fix_data.dateTime_cs" is an integer !!)
        logFile.print(fix_data.dateTime_cs);
      }
      logFile.print(F(";"));
      logFile.print(totalDistance, 3);
      logFile.print(F(";"));
      logFile.print(lapCounter);
      logFile.print(F(";"));
      logFile.print(brake);
      logFile.print(F(";"));
      bitShift = B00000001;
      for (uint8_t i = 0; i < 8; i++) { // We parse the 8 analog values
        tmpComp = bitShift & anaDisabledBits;
        if (tmpComp != bitShift) { // If analog port is enabled ...
          if (i == 6) { // If "Throttle" ...
            logFile.print(throttle);
          } else if (i == 7) { // If "Gear" ...
            logFile.print(gear);
          } else {
            logFile.print(anaValues[i]);
          }
          logFile.print(F(";"));
        }
        bitShift = bitShift << 1;
      }
      logFile.print(bikeRpm);
      logFile.print(F(";"));
      logFile.print(bikeSpeed);
      logFile.print(F(";"));
      logFile.print(fix_data.speed_kph(), 0);
      logFile.print(F(";"));
      logFile.print(fix_data.heading(), 1);
      logFile.print(F(";"));
      logFile.print(event.orientation.y, 0);
      logFile.print(F(";"));
      logFile.print(event.orientation.z, 0);
      logFile.print(F(";"));
      logFile.print(temperature);
      logFile.print(F(";"));
      for (uint8_t i = 0; i < maxMlx; i++) {
        if (mlxAddresses[i] != 0x00) {
          logFile.print(mlxValues[i]);
          logFile.print(F(";"));
        }
      }
      if (addFinishLog == true) {
        logFile.print((posCrossLat / rescaleGPS), 9);
        logFile.print(F(";"));
        logFile.println((posCrossLon / rescaleGPS), 9);
      } else {
        logFile.print(fix_data.latitude(), 9);
        logFile.print(F(";"));
        logFile.println(fix_data.longitude(), 9);
      }
    } else {
      /**************************************************************
        Get bluetooth commands (only when not running)
        Wait for orders
      **************************************************************/
      while (BLUETOOTH_PORT.available()) {
        c = processCharInput(cmdBuffer, BLUETOOTH_PORT.read());
        if (c == '\n') {
          if (strcmp("help", cmdBuffer) == 0) {
            fGetHelp();
          }
          if (strcmp("1", cmdBuffer) == 0) {
            fGetData();
          }
          if (strcmp("2", cmdBuffer) == 0) {
            fGetLastRun();
          }
          if (strcmp("3", cmdBuffer) == 0) {
            fGetToday();
          }
          if (strcmp("4", cmdBuffer) == 0) {
            fGetTrackBest();
          }
          if (strcmp("20", cmdBuffer) == 0) {
            fCalThrottle();
          }
          if (strcmp("21", cmdBuffer) == 0) {
            fCalGear();
          }
          if (strcmp("22", cmdBuffer) == 0) {
            fCal9axis();
          }
          if (strcmp("30", cmdBuffer) == 0) {
            fShowADC();
          }
          if (strcmp("31", cmdBuffer) == 0) {
            fTogAna1();
          }
          if (strcmp("32", cmdBuffer) == 0) {
            fTogAna2();
          }
          if (strcmp("33", cmdBuffer) == 0) {
            fTogAna3();
          }
          if (strcmp("34", cmdBuffer) == 0) {
            fTogAna4();
          }
          if (strcmp("35", cmdBuffer) == 0) {
            fTogAna5();
          }
          if (strcmp("36", cmdBuffer) == 0) {
            fTogAna6();
          }
          if (strcmp("37", cmdBuffer) == 0) {
            fTogAna7();
          }
          if (strcmp("38", cmdBuffer) == 0) {
            fTogAna8();
          }
          if (strcmp("40", cmdBuffer) == 0) {
            fListMlxAddr();
          }
          if (strcmp("41", cmdBuffer) == 0) {
            fClearMlxAddr();
          }
          if (strcmp("42", cmdBuffer) == 0) {
            fAddMlx();
          }
          cmdBuffer[0] = 0;
        }
      }
    }
  }
}
