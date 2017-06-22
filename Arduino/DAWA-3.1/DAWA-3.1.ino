/*
   DAWA 3.0 (Arduino M0)
   Triumph motorbikes datalogger
   Edouard PIGEON - 2017
*/

// Use LCD display ? comment if not
#define USE_DISPLAY

// Use 9 axis ? comment if not
#define USE_BNO055

// Librairies
#include <SPI.h>

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
#ifdef USE_DISPLAY // Choose one of the 3 available LCD type
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
#else
#define LCD_PRINT(x);
#define LCD_INIT;
#define LCD_POS;
#define LCD_CLEAR;
#endif

/*
   9-axis Bosch BNO055 Adafruit library
   https://github.com/adafruit/Adafruit_BNO055
*/
#ifdef USE_BNO055
#include <Adafruit_BNO055.h> // Adafruit 9-axis BNO_055 lib (enable use of BNO055 with "USE_BNO055")
#include <Adafruit_Sensor.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS_B); // USE_BNO055_ADDRESS_B = Adress 0x29 (COM3 pin set to 1)
#endif

/*
   Variable PINs I/O
*/
const uint8_t ledPin = 4; // D4
const uint8_t powerPin = A3; //A3
const uint8_t brakePin = 11; // D11
const uint8_t throttlePin = A1; // A1
const uint8_t gearPin = A0; // A0
const uint8_t sdCsPin = 7; // D7
const uint8_t oledResetPin = 8; // D8

/*
   Vars
*/
int eeptest = 4857;
int eeptest2 = 0;

// Generic
bool interrupt_10hz = 0;
uint8_t runCounter = 0, runCalibration = 0;
uint8_t newRun = 0;
float sec = 0;
char secString[9];
uint8_t i, count;

// Sensors
uint16_t bikeRpm;
uint16_t bikeSpeed;
char bikeRpmString[6];
char bikeSpeedString[6];
char gearvalueString[6];
bool brake = 0;
bool isRunning = 0;
uint16_t throttle;
char throttleString[4];
uint16_t gearValue;
char gear = 'N';
uint8_t gearNCheck = 0;

// RTC
RTC_DS1307 rtc;
uint8_t datetime_array[6] = {1, 1, 17, 13, 37, 0}; // DD/MM/YY HH:MM:SS
#ifdef USE_DISPLAY
char datetime[11];
#endif

// SD Card
File dateFile;
SdFat sd;
SdFile logFile;
char filename[20];
char sdString[50];

// Display
char lcdString[20];
uint8_t lcdLine = 0;

// 9-axis sensor
#ifdef USE_BNO055
char rollString[5];
char pitchString[5];
int8_t temperature;
char temperatureString[4];
uint8_t cal_sys, cal_gyro, cal_accel, cal_mag = 0;
adafruit_bno055_offsets_t calibrationData;
//byte *calibrationBuffer = (byte *) &calibrationData;
#endif

/*
   Triggered when initialization error (program stop and slow blinking led)
*/
void initerror() {
  delay(2000); // Pause 3 sec pour lecture infos LCD
  while (1) {
    digitalWrite(ledPin, digitalRead(ledPin) ^ 1); // Clignotement LED rapide (indique une erreur)
    delay(1000);
  }
}

/*
   Initialisation (I/O, LCD, SD, DATE, 9-AXIS SENSOR, TIMERS/COUNTERS)
*/
void setup() {
  Wire.begin(); // I2C bus init

  // Init I/O pins
  pinMode(ledPin, OUTPUT); // Button light
  pinMode(powerPin, INPUT); // ON/OFF button
  pinMode(brakePin, INPUT); // Brake sensor (0v/12v)
  pinMode(throttlePin, INPUT); // Throttle sensor (analogic from 0v to 5v)
  pinMode(gearPin, INPUT); // Gear sensor (analogic from 0v to 5v)
  pinMode(oledResetPin, OUTPUT); // Reset on OLED screen

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

  // Date setup
  dateFile = sd.open("DATE.TXT", FILE_READ); // If DATE.TXT is found, the date inside is read and updated (format : DD/MM/YY HH:MM:SS)
  if (dateFile) {
    LCD_POS(0, ++lcdLine);
    LCD_PRINT("SET DATE ... ");
    for (i = 0; i < 6; i++) {
      datetime_array[i] = dateFile.parseInt();
    }
    dateFile.close();
    rtc.adjust(DateTime(datetime_array[2], datetime_array[1], datetime_array[0], datetime_array[3], datetime_array[4], datetime_array[5]));
    LCD_PRINT("      OK");

    // Date is setup, we remove the file
    sd.remove("DATE.TXT");
  }

  // Init 9-axis sensor
#ifdef USE_BNO055
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
#endif
  LCD_POS(0, ++lcdLine);
  LCD_PRINT("              READY !");
  delay(2000); // LCD user read time
  LCD_CLEAR;

  /*
      First timer (TC4)
      a 100ms interrupt
      Prescaler = 1, Preload = 4798000, Source clock = 48000000
      Counter on 32 bits > 2x 16 bits counters : TC3 (slave) and TC4 (master)
  */
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TC4_TC5) ; // Register CLKCTRL (Page 119)
  while (GCLK->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // The type cast must fit with the selected timer mode
  TcCount32* TC = (TcCount32*) TC4; // get timer struct

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TC (to enable configuration mode)
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT32;  // Set Timer counter Mode to 32 bits (Page 552)
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; // Set TC as  Match Frq
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1;   // Set prescaler (Page 551)
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // CC0 Register (preload)
  // Resonator accuracy : https://github.com/manitou48/crystals/blob/master/crystals.txt
  // SAMD21 use 1464*32768=47972352Hz
  TC->CC[0].reg = 4798315; // 4798295 looks good / 275 faut ralentir / -50 / +25 / +15 / -10 / +5 / -1 / +1
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  TC->INTENSET.reg = 0;              // disable all interrupts
  TC->INTENSET.bit.MC0 = 1;          // enable compare match to CC0 (P. 622)

  // Enable InterruptVector
  NVIC_EnableIRQ(TC4_IRQn); // interrupt > TC4_Handler(){}

  TC->CTRLA.reg |= TC_CTRLA_ENABLE; // Enable TC (configuration mode finished)
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

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

/*
  100msec interruption (TC3)
*/
void TC4_Handler() {
  TcCount32* TC = (TcCount32*) TC4; // get timer struct
  TC->INTFLAG.bit.MC0 = 1; // writing a one clears the ovf flag of MC0 (P.624)

  // Prepare to read counter
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

  // Long press to start/end the system
  if (digitalRead(A3) == 1) {
    runCounter++;
  } else {
    runCounter = 0;
  }

  interrupt_10hz = 1; // Calculations and SD writing are done outside the interrupt (main loop)
}



// Main loop
void loop() {
  // ON/OFF
  if (runCounter >= 10) { // long press on power button
    runCounter = 0;
    if (isRunning == 1) { // If stopwatch is running ...
      isRunning = 0; // ... we stop recording
      digitalWrite(ledPin, 0); // LED off
      logFile.close();
      LCD_CLEAR;
    } else {
      isRunning = 1; // ... we start recording
      newRun = 1; // A new file will be created
      LCD_CLEAR;
    }
  }

  if (isRunning == 1) {
    if (newRun == 1) { // We create a new file
      newRun = 0;
      DateTime now = rtc.now(); // Get date now
      sprintf(filename, "%02u%02u%02u-%02u%02u%02u.txt", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
      if (logFile.open(filename, O_CREAT | O_WRITE | O_EXCL)) {
        logFile.println("s;brake;throttle;gear;rpm;speed;roll;pitch;temp"); // Header of the new file
        logFile.println("   0.000;0;0;N;0;   0;   0;0"); // First "0" line
        rtc.adjust(DateTime(2017,01,01,0,0,0));
        delay(100);
        sec = 0; // Reset stopwatch
      } else {
        initerror();
      }
    }

    if (interrupt_10hz == 1) { // A 100ms interrupt was triggered, we do here some maths for printing clean data (LCD + SD card)
      brake = digitalRead(brakePin); // Read "brakePin" (pin is plugged on the "+" of the stop light)
      throttle = min(((analogRead(throttlePin) / 10) * 1.23), 100); // Read voltage on "throttlePin"
      gearValue = analogRead(gearPin); // Read voltage on "gearPin" : every gear output a specific voltage (raw data measured : 314, 520, 660, 787, 888, 975, 1023 (N))
      interrupt_10hz = 0;
      sec = sec + 0.1; // @10Hz (100ms interrupt) > increments of 0.100s
      digitalWrite(ledPin, digitalRead(ledPin) ^ 1); // Blinking LED (user knows it's running)

#ifdef USE_BNO055
      // 9-axis sensor (roll/pitch)
      sensors_event_t event;
      bno.getEvent(&event);
      dtostrf(event.orientation.y, 4, 0, rollString); // Roll
      dtostrf(event.orientation.z, 4, 0, pitchString); // Pitch
      temperature = bno.getTemp(); // Temperature
      bno.getCalibration(&cal_sys, &cal_gyro, &cal_accel, &cal_mag); // Calibration status
#endif

      // Gear selected
      if (gearValue < 366) {
        gear = '1';
      }
      if (gearValue > 500 && gearValue < 540) {
        gear = '2';
      }
      if (gearValue > 640 && gearValue < 680) {
        gear = '3';
      }
      if (gearValue > 767 && gearValue < 807) {
        gear = '4';
      }
      if (gearValue > 868 && gearValue < 908) {
        gear = '5';
      }
      if (gearValue > 955 && gearValue < 995) {
        gear = '6';
      }
      if (gearValue > 1000) { // "N" = 1023
        if (gearNCheck > 3) { // We test 3 times to prevent displaying "N" between 2 gears
          gear = 'N';
        } else {
          gearNCheck++;
        }
      } else {
        gearNCheck = 0;
      }

      // Format strings
      dtostrf(sec, 8, 1, secString); // !! float conversion !!
      sprintf(bikeRpmString, "%5u", bikeRpm);
      sprintf(bikeSpeedString, "%5u", bikeSpeed);
      sprintf(gearvalueString, "%5u", gearValue);
      sprintf(throttleString, "%3u", throttle);
#ifdef USE_BNO055
      sprintf(temperatureString, "%d", temperature);
      sprintf(sdString, "%s;%d;%u;%c;%u;%u;%s;%s;%s", secString, brake, throttle, gear, bikeRpm, bikeSpeed, rollString, pitchString, temperatureString);
#else
      sprintf(sdString, "%s;%d;%u;%c;%u", secString, brake, throttle, gear, bikeRpm);
#endif
      //logFile.println(sdString); // Writing a new line in the file
    }

    /*
       Displaying on LCD (USE_DISPLAY)
       Everything is optional
    */
#ifdef USE_DISPLAY
    lcdLine = 0;
    LCD_POS(0, lcdLine);
    DateTime now = rtc.now();
    sprintf(datetime, "%02u/%02u/%u", now.day(), now.month(), now.year());
    LCD_PRINT(datetime);
    LCD_POS(78, lcdLine);
    sprintf(datetime, "%02u:%02u:%02u", now.hour(), now.minute(), now.second());
    LCD_PRINT(datetime);
    LCD_POS(0, ++lcdLine);
    sprintf(lcdString, "TIME:%16s", secString);
    LCD_PRINT(lcdString);
    LCD_POS(0, ++lcdLine);
    sprintf(lcdString, "RPM:%11utr/min", bikeRpm);
    LCD_PRINT(lcdString);
    LCD_POS(0, ++lcdLine);
    sprintf(lcdString, "SPEED:%11ukm/h", bikeSpeed);
    LCD_PRINT(lcdString);
    LCD_POS(0, ++lcdLine);
    sprintf(lcdString, "GEAR:%16c", gear);
    //sprintf(lcdString, "GEAR:%11c/%u", gear, gearValue);
    LCD_PRINT(lcdString);
    LCD_POS(0, ++lcdLine);
    sprintf(lcdString, "BRAKE:%15d", brake);
    LCD_PRINT(lcdString);
    LCD_POS(0, ++lcdLine);
    sprintf(lcdString, "THROTTLE:%12u", throttle);
    LCD_PRINT(lcdString);
    LCD_POS(0, ++lcdLine);
    sprintf(lcdString, "TEMP:%16s", temperatureString);
    LCD_PRINT(lcdString);
#endif
  } else {
#ifdef USE_BNO055
    lcdLine = 0;

    // Date
    LCD_POS(0, lcdLine);
    DateTime now = rtc.now();
    sprintf(datetime, "%02u/%02u/%u", now.day(), now.month(), now.year());
    LCD_PRINT(datetime);
    LCD_POS(78, lcdLine);
    sprintf(datetime, "%02u:%02u:%02u", now.hour(), now.minute(), now.second());
    LCD_PRINT(datetime);

    // Calibration offsets 
    /*bno.getSensorOffsets(calibrationData);
    LCD_POS(0, lcdLine);
    LCD_PRINT("OFFSETS:");
    LCD_POS(0, ++lcdLine);
    sprintf(lcdString, "%05u;%05u;%05u", calibrationData.accel_offset_x, calibrationData.accel_offset_x, calibrationData.accel_offset_z);
    LCD_PRINT(lcdString);
    LCD_POS(0, ++lcdLine);
    sprintf(lcdString, "%05u;%05u;%05u", calibrationData.gyro_offset_x, calibrationData.gyro_offset_y, calibrationData.gyro_offset_z);
    LCD_PRINT(lcdString);
    LCD_POS(0, ++lcdLine);
    sprintf(lcdString, "%05u;%05u;%05u", calibrationData.mag_offset_x, calibrationData.mag_offset_y, calibrationData.mag_offset_z);
    LCD_PRINT(lcdString);
    LCD_POS(0, ++lcdLine);
    sprintf(lcdString, "%05u;%05u", calibrationData.accel_radius, calibrationData.mag_radius);
    LCD_PRINT(lcdString);*/

    // Calibration status
    bno.getCalibration(&cal_sys, &cal_gyro, &cal_accel, &cal_mag); // Get calibration status
    LCD_POS(0, ++lcdLine);
    sprintf(lcdString, "CALIB:%9d;%d;%d;%d", cal_sys, cal_gyro, cal_accel, cal_mag);
    LCD_PRINT(lcdString);

    // Roll/pitch
    sensors_event_t event;
    bno.getEvent(&event);
    dtostrf(event.orientation.y, 4, 0, rollString); // Roll
    dtostrf(event.orientation.z, 4, 0, pitchString); // Pitch
    LCD_POS(0, ++lcdLine);
    sprintf(lcdString, "ROLL/PITCH:%5s/%4s", rollString, pitchString);
    LCD_PRINT(lcdString);

    //Temperature
    LCD_POS(0, ++lcdLine);
    sprintf(lcdString, "TEMP:%16d", bno.getTemp());
    LCD_PRINT(lcdString);
    delay(100);
#endif
  }
}
