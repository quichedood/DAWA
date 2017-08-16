/*
   DAWA 3.0 (Arduino M0)
   Triumph motorbikes datalogger
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
   Maxim DS13xx library
   https://github.com/adafruit/RTClib
*/
#include <RTClib.h>

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
   LCD libraries
   Choose one of the 2 available LCD type
   I2C ASCII OLED : https://github.com/greiman/SSD1306Ascii
   SPI ASCII OLED : https://github.com/greiman/SSD1306Ascii (faster)
*/

// I2C ASCII OLED libraries
/*#include "SSD1306Ascii.h" // OLED LCD based on SSD1306 chip, ASCII only no graphics (Official Github : https://github.com/greiman/SSD1306Ascii)
  #include "SSD1306AsciiWire.h"
  SSD1306AsciiWire oled; // 128x64 LCD OLED
  #define I2C_ADDRESS 0x3C // 128x64 LCD OLED (I2C Address definition)*/

// I2C ASCII OLED aliases definition
/*#define LCD_PRINT(x)   oled.print(x);
  #define LCD_INIT       oled.begin(&Adafruit128x64, I2C_ADDRESS);oled.setFont(System5x7);
  #define LCD_POS(x,y)   oled.setCursor(x, y);
  #define LCD_CLEAR      oled.clear();*/

// SPI ASCII OLED libraries
#include "SSD1306Ascii.h" // OLED LCD based on SSD1306 chip, ASCII only no graphics (Official Github : https://github.com/greiman/SSD1306Ascii)
#include "SSD1306AsciiSpi.h"
SSD1306AsciiSpi oled; // 128x64 LCD OLED
#define OLED_DC 9
#define OLED_CS 13

// SPI ASCII OLED aliases definition
#define LCD_PRINT(x)   oled.print(x);
#define LCD_INIT       oled.begin(&Adafruit128x64, OLED_CS, OLED_DC);oled.setFont(System5x7);
#define LCD_POS(x,y)   oled.setCursor(x, y);
#define LCD_CLEAR      oled.clear();

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

/*
   Vars
*/
uint32_t lastPinRead = 0;
uint32_t lastLCDupdate = 0;
uint32_t lastSdSync = 0;
uint32_t stopwatchStartTime = 0;
uint32_t elapsedTime = 0;
//float brt;
bool gpsDataReady = false;
bool sensorsDataReady = false;
//char brts[8];


// Generic
uint8_t runCalibration = 0;
uint8_t newRun = 0;
float sec = 0.000;
char secString[11];
uint8_t i, count;

// GPS
char latString[15];
char longString[15];
uint16_t nowms, nowms2;

// Sensors
uint16_t bikeRpm;
float bikeRpmCoeff = 1.05;
char bikeRpmString[6];
uint16_t bikeSpeed;
char bikeSpeedString[6];
uint16_t gearValue;
char gear = 'N';
uint8_t gearNCheck = 0;
char gearvalueString[6];
uint8_t brake = 0;
char brakeString[4];
bool isRunning = false;
uint16_t throttle;
char throttleString[4];

// RTC
RTC_DS1307 rtc;
char datetime[11];

// SD Card
File dateFile;
SdFat sd;
SdFile logFile;
char filename[20];
char sdString[86]; //76

// Display
char lcdString[20];
uint8_t lcdLine = 0;

// 9-axis sensor
char rollString[5];
char pitchString[5];
int8_t temperature;
char temperatureString[4];
uint8_t cal_sys, cal_gyro, cal_accel, cal_mag = 0;
adafruit_bno055_offsets_t calibrationData;
sensors_event_t event;
//byte *calibrationBuffer = (byte *) &calibrationData;

/*
   Triggered when initialization error (program stop and slow blinking led)
*/
void initerror() {
  delay(2000); // Pause 3 sec pour lecture infos LCD
  while (1) {
    digitalWrite(ledPin, digitalRead(ledPin) ^ 1); // Clignotement LED rapide (indique une erreur)
    delay(200);
  }
}

/*
   Send UBX commands to UBLOX GPS
*/
void sendUBX( const unsigned char *progmemBytes, size_t len ) {
  gps_port.write( 0xB5 ); // SYNC1
  gps_port.write( 0x62 ); // SYNC2

  uint8_t a = 0, b = 0;
  while (len-- > 0) {
    uint8_t c = pgm_read_byte( progmemBytes++ );
    a += c;
    b += a;
    gps_port.write( c );
  }

  gps_port.write( a ); // CHECKSUM A
  gps_port.write( b ); // CHECKSUM B
}

void extIntSys() {
  //TcCount32* TC = (TcCount32*) TC4; // get timer struct
  //TC->INTFLAG.bit.MC0 = 1; // writing a one clears the ovf flag of MC0 (P.624)


}

/*
   Initialisation (I/O, LCD, SD, DATE, 9-AXIS SENSOR, TIMERS/COUNTERS)
*/
void setup() {
  Wire.begin(); // I2C bus init

  // GPS Init
  gps_port.begin(9600); // Start the UART for the GPS device (you cannot block for more than "buffer size" * 11 / 9600 = 70ms)
  sendUBX(ubxRate10Hz, sizeof(ubxRate10Hz)); // Set refresh rate to 10Hz
  sendUBX(ubxTimepulse, sizeof(ubxTimepulse)); // Set timepulse output ON

  // Init I/O pins
  pinMode(ledPin, OUTPUT); // Button light
  pinMode(powerPin, INPUT); // ON/OFF button
  pinMode(brakePin, INPUT); // Brake sensor (0v/12v)
  pinMode(throttlePin, INPUT); // Throttle sensor (analogic from 0v to 5v)
  pinMode(gearPin, INPUT); // Gear sensor (analogic from 0v to 5v)
  pinMode(oledResetPin, OUTPUT); // Reset on OLED screen
  pinMode(timepulsePin, INPUT);

  // Debounce power button
  debouncer.attach(powerPin);
  debouncer.interval(2000); // interval in ms

  //rtc.writeSqwPinMode(SquareWave4kHz);

  // Init + clear LCD (+Backlight)
  digitalWrite(oledResetPin, HIGH); // Reset pin on OLED screen should be tied to +VCC
  LCD_INIT;
  delay(1000); // LCD init delay
  LCD_CLEAR;

  // Init EEPROM
  LCD_POS(0, lcdLine);
  LCD_PRINT("EEPROM:");
  if (eep.begin(twiClock400kHz) == 0) {
    LCD_PRINT("            OK");
  } else {
    LCD_PRINT("        FAILED");
    initerror();
  }

  // Init SD card
  LCD_POS(0, ++lcdLine);
  LCD_PRINT("SD:");
  if (!sd.begin(sdCsPin, SD_SCK_MHZ(50))) {
    LCD_PRINT("            FAILED");
    initerror();
  }
  LCD_PRINT("                OK");

  // Init 9-axis sensor
  LCD_POS(0, ++lcdLine);
  LCD_PRINT("9-AXIS:");
  if (!bno.begin()) {
    LCD_PRINT("        FAILED");
    initerror();
  }
  LCD_PRINT("            OK");
  bno.setExtCrystalUse(true); // Use external quartz for 9-axis sensor
  LCD_POS(0, ++lcdLine);
  LCD_PRINT(">> PRESS TO CALIB. <<");
  digitalWrite(ledPin, HIGH); // We light up the led for the user (calibration is possible)
  delay(2000); // LCD user read time
  if (digitalRead(A3) == 1) { // If user press button, we start calibration
    LCD_POS(0, ++lcdLine);
    LCD_PRINT("CALIBRATE:");
    while (!bno.isFullyCalibrated()) { // We wait until sensor fully calibrated !! Offsets are returned only if sensor is calibrated !!
      delay(200);
      bno.getCalibration(&cal_sys, &cal_gyro, &cal_accel, &cal_mag); // Get calibration state (from 0 to 3 for the 3 sensors + main calibration)
      sprintf(lcdString, "%d;%d;%d;%d", cal_sys, cal_gyro, cal_accel, cal_mag);
      LCD_POS(84, lcdLine);
      LCD_PRINT(lcdString);
      delay(200);
    }
    bno.getSensorOffsets(calibrationData); // Sensor is calibrated, we save offsets to var (calibrationData)
    if (EEPROM_writeAnything(0, calibrationData) == sizeof(adafruit_bno055_offsets_t)) { // We save the 11 offsets to eeprom
      LCD_PRINT("  OK");
    } else {
      initerror();
    }
  } else { // Calibration not asked
    LCD_POS(0, ++lcdLine);
    LCD_PRINT("LOAD CALIB.");
    if (EEPROM_readAnything(0, calibrationData) == sizeof(adafruit_bno055_offsets_t)) { // We try loading offsets from EEPROM
      bno.setSensorOffsets(calibrationData); // We set them in var
      LCD_PRINT("        OK");
    } else {
      LCD_PRINT("   NO DATA");
      initerror(); // Calibration not asked and no offset in EEPROM > can't go further !
    }
  }
  digitalWrite(ledPin, LOW); // Calibration is done, LED off

  // End Init
  LCD_POS(0, ++lcdLine);
  LCD_PRINT("              READY !");
  delay(2000); // LCD user read time
  LCD_CLEAR;

  /*
      A 100ms interrupt based on GPS Ublox timepulse signal // Call "extIntSys" on each interruption
  */
  //attachInterrupt(digitalPinToInterrupt(timepulsePin), extIntSys, FALLING);

  /*
      2 counters (TCC0 & TCC1)
      form SPEED and RPM values

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
      LCD_CLEAR;
    } else {
      isRunning = true; // ... we start recording
      stopwatchStartTime = millis(); // Define stopwatch start time
      if (fix_data.valid.time) {
        rtc.adjust(DateTime(fix_data.dateTime.year, fix_data.dateTime.month, fix_data.dateTime.date, fix_data.dateTime.hours, fix_data.dateTime.minutes, fix_data.dateTime.seconds)); // We adjust RTC clock with GPS datetime (UTC only)
      }
      DateTime now = rtc.now(); // Get date now
      sprintf(filename, "%02u%02u%02u-%02u%02u%02u.txt", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
      if (logFile.open(filename, O_CREAT | O_WRITE | O_EXCL)) {
        logFile.println("s;brake;throttle;gear;rpm;speed;roll;pitch;temp;lat;long"); // Header of the new file
      } else {
        initerror();
      }

      // Print static string to reduce time consumming
      LCD_CLEAR;
      LCD_POS(0, 0);
      LCD_PRINT("Running ...");
    }
  }

  // Every ~100ms (10Hz), we get all values (RPM, Speed, brake, gear, 9-axis data)
  elapsedTime = millis() - lastPinRead;
  if (elapsedTime >= 100) {
    lastPinRead = millis(); // Reset last pin read
    //brt = elapsedTime / 100.000;

    if (isRunning) {
      sec = (millis() - stopwatchStartTime) / 1000.000; // Update stopwatch time
    } else {
      sec = 0.000;
    }

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
    temperature = bno.getTemp(); // Temperature
    //bno.getCalibration(&cal_sys, &cal_gyro, &cal_accel, &cal_mag); // Calibration status

    // Format strings
    //dtostrf(brt, 6, 3, brts);
    dtostrf(event.orientation.y, 4, 0, rollString); // Roll
    dtostrf(event.orientation.z, 4, 0, pitchString); // Pitch
    dtostrf(sec, 10, 3, secString); // !! float conversion !!
    dtostrf(fix_data.latitude(), 13, 9, latString);
    dtostrf(fix_data.longitude(), 13, 9, longString);
    sprintf(bikeRpmString, "%5u", bikeRpm);
    sprintf(bikeSpeedString, "%5u", bikeSpeed);
    sprintf(gearvalueString, "%5u", gearValue);
    sprintf(throttleString, "%3u", throttle);
    sprintf(brakeString, "%1u", brake);
    sprintf(temperatureString, "%d", temperature);

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

  // Get GPS frames through serial port (this is CRITICAL, data could be sent at any moment by the GPS so main loop should be executed in a minimum of time)
  if (gps.available(gps_port)) {
    fix_data = gps.read();
    gpsDataReady = true;
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
    lastLCDupdate = millis();
    if (isRunning) {
      digitalWrite(ledPin, digitalRead(ledPin) ^ 1); // Blinking LED (user knows it's running)
    } else {     
      lcdLine = 0;
      LCD_POS(0, lcdLine);
      DateTime now = rtc.now();
      sprintf(datetime, "%02u/%02u/%u", now.day(), now.month(), now.year());
      LCD_PRINT(datetime);
      LCD_POS(78, lcdLine);
      sprintf(datetime, "%02u:%02u:%02u", now.hour(), now.minute(), now.second());
      LCD_PRINT(datetime);
      LCD_POS(0, ++lcdLine);
      sprintf(lcdString, "RPM:%11str/min", bikeRpmString);
      //sprintf(lcdString, "TIME:%16s", secString);
      LCD_PRINT(lcdString);
      LCD_POS(0, ++lcdLine);
      sprintf(lcdString, "SPEED:%11skm/h", bikeSpeedString);
      LCD_PRINT(lcdString);
      LCD_POS(0, ++lcdLine);
      sprintf(lcdString, "GEAR:%16c", gear);
      //sprintf(lcdString, "GEAR:%11c/%u", gear, gearValue);
      LCD_PRINT(lcdString);
      LCD_POS(0, ++lcdLine);
      sprintf(lcdString, "BRAKE:%15s", brakeString);
      LCD_PRINT(lcdString);
      LCD_POS(0, ++lcdLine);
      sprintf(lcdString, "THROTTLE:%12s", throttleString);
      LCD_PRINT(lcdString);
      LCD_POS(0, ++lcdLine);
      sprintf(lcdString, "TEMP:%16s", temperatureString);
      LCD_PRINT(lcdString);
      LCD_POS(0, ++lcdLine);
      sprintf(lcdString, "GPS:%02u:%02u:%02u:%03u", fix_data.dateTime.hours, fix_data.dateTime.minutes, fix_data.dateTime.seconds, fix_data.dateTime_cs);
      LCD_PRINT(lcdString);
    }
  }

  if (isRunning) {
    if (gpsDataReady && sensorsDataReady) {
      //write pins, IMU, GPS to SD card
      sprintf(sdString, "%s;%s;%s;%c;%s;%s;%s;%s;%s;%s;%s;%02u;%03u", secString, brakeString, throttleString, gear, bikeRpmString, bikeSpeedString, rollString, pitchString, temperatureString, latString, longString, fix_data.dateTime.seconds, fix_data.dateTime_cs);
      logFile.println(sdString); // Writing a new line in the file !! Log file should be closed properly for data to be saved !!
      gpsDataReady = false;
      sensorsDataReady = false;
    }
  }
}
