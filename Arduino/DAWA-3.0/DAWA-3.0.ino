/*
 * DAWA 3.0 (Arduino M0)
 * Triumph motorbikes datalogger
 * Edouard PIGEON - 2017
 */

// Use LCD display ? comment if not
#define USE_DISPLAY

// Use 9 axis ? comment if not
#define USE_BNO055

// Librairies
//#include <Streaming.h>
#include <SPI.h>
#include <SD.h>

/*
   Maxim DS13xx library
   https://github.com/adafruit/RTClib
*/
#include <RTClib.h> // Maxim DS13xx library

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

   Choose one of the 3 available LCD type
   SERIAL LCD (default included)
   I2C LCD : https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library
   I2C ASCII OLED : https://github.com/greiman/SSD1306Ascii
*/
#ifdef USE_DISPLAY // Choose one of the 3 available LCD type
// I2C ASCII OLED libraries
/*#include "SSD1306Ascii.h" // OLED LCD based on SSD1306 chip, ASCII only no graphics (Official Github : https://github.com/greiman/SSD1306Ascii)
  #include "SSD1306AsciiWire.h"
  SSD1306AsciiWire oled; // 128x64 LCD OLED
  #define I2C_ADDRESS 0x3C // 128x64 LCD OLED (I2C Address definition)*/

// SPI ASCII OLED libraries
#include "SSD1306Ascii.h" // OLED LCD based on SSD1306 chip, ASCII only no graphics (Official Github : https://github.com/greiman/SSD1306Ascii)
#include "SSD1306AsciiSpi.h"
SSD1306AsciiSpi oled; // 128x64 LCD OLED
#define OLED_DC 9
#define OLED_CS 13

// I2C ASCII OLED aliases definition
/*#define LCD_PRINT(x)   oled.print(x);
  #define LCD_INIT       oled.begin(&Adafruit128x64, I2C_ADDRESS);oled.setFont(System5x7);
  #define LCD_POS(x,y)   oled.setCursor(x, y);
  #define LCD_CLEAR      oled.clear();*/

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
const int ledPin = 4; // D4
const int powerPin = A3; //A3
const int brakePin = 11; // D11
const int throttlePin = A1; // A1
const int gearPin = A0; // A0
const uint8_t sdCsPin = 7; // D8
const uint8_t oledResetPin = 8; // D7

/*
   Var
*/
uint16_t bikeRpm;
uint16_t bikeSpeed;
uint8_t lcdLine = 0;
char bikeRpmString[6];
char bikeSpeedString[6];
char gearvalueString[6];
bool brake = 0;
bool isRunning = 0;
bool interrupt_10hz = 0;
uint8_t runCounter = 0, runCalibration = 0;
uint8_t newRun = 0;
uint16_t throttle;
char throttleString[4];
uint16_t gearValue;
char gear = 'N';
char filename[12];
File dataFile;
char sdString[50];
float sec = 0;
char secString[9];
uint8_t gearNCheck = 0;
uint8_t i, count;
RTC_DS1307 rtc;
uint8_t datetime_array[6] = {1, 1, 17, 13, 37, 0}; // 01/01/17 13:37:00
#ifdef USE_BNO055
char rollString[5];
char pitchString[5];
int8_t temperature;
char temperatureString[4];
uint8_t cal_sys, cal_gyro, cal_accel, cal_mag = 0;
char lcdString[20];
adafruit_bno055_offsets_t calibrationData;
//byte *calibrationBuffer = (byte *) &calibrationData;
#endif
#ifdef USE_DISPLAY
char datetime[11];
#endif


// Triggered when initialization error (program stop and slow blinking led)
void initerror() {
  delay(2000); // Pause 3 sec pour lecture infos LCD
  while (1) {
    digitalWrite(ledPin, digitalRead(ledPin) ^ 1); // Clignotement LED rapide (indique une erreur)
    delay(1000);
  }
}

// Initialisation
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
    LCD_PRINT("OK");
  } else {
    LCD_PRINT("FAILED");
    initerror();
  }

  // Init SD card
  LCD_POS(0, ++lcdLine);
  LCD_PRINT("SD:");
  if (!SD.begin(sdCsPin)) {
    LCD_PRINT("FAILED");
    initerror();
  }
  LCD_PRINT("OK");

  // Date setup
  dataFile = SD.open("DATE.TXT", FILE_READ); // If DATE.TXT is found, the date inside is read and updated (format : 24/04/17 19:44:30)
  if (dataFile) {
    LCD_POS(0, ++lcdLine);
    LCD_PRINT("SET DATE ... ");
    for (i = 0; i < 6; i++) {
      datetime_array[i] = dataFile.parseInt();
    }
    dataFile.close();
    rtc.adjust(DateTime(datetime_array[2], datetime_array[1], datetime_array[0], datetime_array[3], datetime_array[4], datetime_array[5]));

    LCD_PRINT("OK");

    // On supprime le fichier contenant la date à définir
    SD.remove("DATE.TXT");
  }
  
  // Init 9-axis sensor
#ifdef USE_BNO055
  LCD_POS(0, ++lcdLine);
  LCD_PRINT("9AXIS:");
  if (!bno.begin()) {
    LCD_PRINT("FAILED");
    initerror();
  }
  LCD_PRINT("OK");

  // Use external quartz for 9-axis sensor
  bno.setExtCrystalUse(true);

  // Temporisation pour lecture des éventuels codes erreur sur LCD
  LCD_POS(0, ++lcdLine);
  LCD_PRINT(">> PRESS TO CALIB. <<");
  digitalWrite(ledPin, HIGH); // On allume la LED pour prevenir la possibilité d'effectuer une calibration
  delay(4000); // Pause 2 sec, le temps à l'utilisateur de demander la calibration si souhaité

  // Si l'utilisateur appuie sur le bouton, on démarre la calibration
  if (digitalRead(A3) == 1) {
    // Calibration demandée
    LCD_POS(0, ++lcdLine);
    LCD_PRINT("CALIBRATION ... ");

    // On attend la calibration complète pour mémoriser les offsets !! Les offsets ne sont retournés que si la calibration est complète !!
    while (!bno.isFullyCalibrated()) {
      delay(200); // Pause 0.2 sec
      bno.getCalibration(&cal_sys, &cal_gyro, &cal_accel, &cal_mag); // On récupère l'état de la calibration (de 0 à 3 pour les 3 capteurs + calibration générale)
      sprintf(lcdString, "%d;%d;%d;%d", cal_sys, cal_gyro, cal_accel, cal_mag); // Mise en forme pour affichage LCD
      LCD_POS(90, lcdLine);
      LCD_PRINT(lcdString); // Affichage de l'état de la calibration sur la 2ème ligne du LCD
      delay(200); // Pause 0.2 sec
    }

    // Le capteur est calibré, on stocke les bons offsets dans la structure prévue à cet effet (calibrationData)
    bno.getSensorOffsets(calibrationData);

    // On écrit les 11 offsets sur l'EEPROM
    if (EEPROM_writeAnything(0, calibrationData) == sizeof(adafruit_bno055_offsets_t)) {
      LCD_POS(0, ++lcdLine);
      LCD_PRINT("CALIB. OK");
    } else {
      initerror();
    }
  } else {
    // Calibration non demandée
    LCD_POS(0, ++lcdLine);
    LCD_PRINT("LOAD CALIB. ... ");

    // On essaie de charger les 11 valeurs depuis l'EEPROM
    if (EEPROM_readAnything(0, calibrationData) == sizeof(adafruit_bno055_offsets_t)) {
      bno.setSensorOffsets(calibrationData); // On définit les offsets avec les données récupérées du fichier
      LCD_PRINT("OK");
    } else { // Si aucun fichier de calibration
      LCD_PRINT("NO DATA");
      initerror(); // On considère que c'est une erreur, risque de données faussées
    }
  }

  // Calibration terminé, on éteint la LED
  digitalWrite(ledPin, LOW);
#endif
  LCD_POS(0, ++lcdLine);
  LCD_PRINT("READY !");
  delay(2000); // Pause 1 sec, le temps à l'utilisateur de lire les infos lcd
  LCD_CLEAR;

  /*
      First timer (TC3)
      a 100ms interrupt
      Prescaler = 256, Preload = 18730
  */

  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | 0x1B) ;
  while (GCLK->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // The type cast must fit with the selected timer mode
  TcCount16* TC = (TcCount16*) TC3; // get timer struct

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TC (to enable configuration mode)
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;  // Set Timer counter Mode to 16 bits
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; // Set TC as  Match Frq
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV256;   // Set prescaler
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
  TC->CC[0].reg = 18730; // 18749>trop lent / 18740>trop lent / 18730>trop rapide ?
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  TC->INTENSET.reg = 0;              // disable all interrupts
  TC->INTENSET.bit.MC0 = 1;          // enable compare match to CC0

  // Enable InterruptVector
  NVIC_EnableIRQ(TC3_IRQn); // interrupt > TC3_Handler(){}

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

// Interruption 100msec (TC3)
void TC3_Handler() {
  TcCount16* TC = (TcCount16*) TC3; // get timer struct
  //if (TC->INTFLAG.bit.MC0 == 1) {
  TC->INTFLAG.bit.MC0 = 1; // writing a one clears the flag ovf flag

  // Prepare to read counter
  REG_TCC0_CTRLBSET = TCC_CTRLBSET_CMD_READSYNC;  // Trigger a read synchronization on the COUNT register
  while (TCC0->SYNCBUSY.bit.CTRLB);               // Wait for the CTRLB register write synchronization
  while (TCC0->SYNCBUSY.bit.COUNT);               // Wait for the COUNT register read sychronization
  bikeSpeed = REG_TCC0_COUNT;                // Lecture registre TCNT0 (compteur du timer0). Aucun débordement possible au vu du signal sorti par le Daytona (75 pulses/125ms @ 1200tr/min ou 5 pulse/100tr/min (100msec))
  REG_TCC0_COUNT = 0x0000;                        // Clear timer's COUNT value
  while (TCC0->SYNCBUSY.bit.COUNT);               // Wait for synchronization
  
  REG_TCC1_CTRLBSET = TCC_CTRLBSET_CMD_READSYNC;  // Trigger a read synchronization on the COUNT register
  while (TCC1->SYNCBUSY.bit.CTRLB);               // Wait for the CTRLB register write synchronization
  while (TCC1->SYNCBUSY.bit.COUNT);               // Wait for the COUNT register read sychronization
  bikeRpm = REG_TCC1_COUNT;                  // Lecture registre TCNT1 (compteur du timer1). Aucun débordement possible au vu du signal sorti par le Daytona (60 pulses/100ms @ 1200tr/min ou 5 pulse/100tr/min (100msec))
  REG_TCC1_COUNT = 0x0000;                        // Clear timer's COUNT value
  while (TCC1->SYNCBUSY.bit.COUNT);               // Wait for synchronization
  
  // Appui long pour arrêter/démarrer la mesure
  if (digitalRead(A3) == 1) {
    runCounter++;
  } else {
    runCounter = 0;
  }

  // Les calculs et écriture sur carte SD se font en dehors de l'interruption (cf main loop)
  interrupt_10hz = 1;
}

// Programme principal
void loop() {
  // ON/OFF
  if (runCounter >= 10) { // Appui long sur bouton power
    runCounter = 0;
    if (isRunning == 1) { // Si l'enregistrement était en cours ...
      isRunning = 0; // ... on arrête l'enregistrement
      digitalWrite(ledPin, 0); // On éteint la LED indiquant l'aquisition de données en cours
      dataFile.close(); // On ferme le fichier
    } else {
      isRunning = 1; // ... sinon on démarre l'enregistrement
      newRun = 1; // Permet la création d'un nouveau fichier sur la carte SD
    }
  }

  if (isRunning == 1) {
    if (newRun == 1) { // On créé un nouveau fichier et on enregistre
      newRun = 0;
      DateTime now = rtc.now();
      sprintf(filename, "%02u%02u%02u.txt", now.day(), now.hour(), now.minute()); // Limite format nom de fichier "8.3"
      dataFile = SD.open(filename, FILE_WRITE); // Ouverture fichier sur carte SD
      //if(dataFile) {
      dataFile.println("s;brake;throttle;gear;rpm;speed;roll;pitch;temp"); // Ecriture entêtes fichier
      dataFile.println("   0.000;0;0;N;0;   0;   0;0"); // Ecriture première ligne fichier
      sec = 0; // Reset chrono
      //} else {
      //  isRunning = 0;
      //  digitalWrite(D4, 0); // On éteint la LED indiquant l'aquisition de données en cours
      //  }
    }

    // L'interruption vient d'avoir eu lieu, on inscrit les données sur la carte SD après calcul & mise en forme
    if (interrupt_10hz == 1) {
      brake = digitalRead(brakePin); // Lecture signal "brakePin" (banchement sur l'alimentation du feu stop avec diviseur de tension pour ramener à 0/+5v)
      throttle = min(((analogRead(throttlePin) / 10) * 1.23), 100); // Lecture tension port "throttlePin" : la tension varie en fonction de l'ouverture de la poignée de gaz (valeurs brutes mesurées : 137 à 820 / ratio 1.23)
      gearValue = analogRead(gearPin); // Lecture tension port "gearPin" : chaque rapport sort une tension différente (valeurs brutes mesurées : 314, 520, 640, 774, 880, 960, 1023 (N))
  
      interrupt_10hz = 0;
      sec = sec + 0.1; // @10Hz > pas de 0.100s
      digitalWrite(ledPin, digitalRead(ledPin) ^ 1); // Clignotement LED (indicateur bon fonctionnement)

#ifdef USE_BNO055
      // Capteur position (roll/pitch) - Précision au degré
      sensors_event_t event;
      bno.getEvent(&event);
      dtostrf(event.orientation.y, 4, 0, rollString); // Roll
      dtostrf(event.orientation.z, 4, 0, pitchString); // Pitch
      temperature = bno.getTemp(); // Température ambiante
      bno.getCalibration(&cal_sys, &cal_gyro, &cal_accel, &cal_mag); // Etat de calibration capteur 9 axes (facultatif)
#endif

      // Calcul rapport selectionné
      if (gearValue < 366) { // Sélection vitesse en fonction de la tension mesurée sur port analogique
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
        if (gearNCheck > 3) { // On teste 3 fois de suite pour éviter d'afficher "N" entre 2 passages de vitesse
          gear = 'N';
        } else {
          gearNCheck++;
        }
      } else {
        gearNCheck = 0;
      }

      // Mise en forme chaines de caractères
      dtostrf(sec, 8, 3, secString); // Préparation chaine pour affichage SD/LCD (conversion float impossible avec sprintf)
      sprintf(bikeRpmString, "%5u", bikeRpm); // Préparation chaine pour affichage SD/LCD
      sprintf(bikeSpeedString, "%5u", bikeSpeed); // Préparation chaine pour affichage SD/LCD
      sprintf(gearvalueString, "%5u", gearValue); // Préparation chaine pour affichage SD/LCD (valeurs source gear)
      sprintf(throttleString, "%3u", throttle); // Préparation chaine pour affichage SD/LCD
#ifdef USE_BNO055
      sprintf(temperatureString, "%d", temperature); // Préparation chaine pour affichage SD/LCD
      sprintf(sdString, "%s;%d;%u;%c;%u;%u;%s;%s;%s", secString, brake, throttle, gear, bikeRpm, bikeSpeed, rollString, pitchString, temperatureString); // Préparation chaine pour affichage SD/LCD
#else
      sprintf(sdString, "%s;%d;%u;%c;%u", secString, brake, throttle, gear, bikeRpm); // Préparation chaine pour affichage SD/LCD
#endif

      // Ecriture carte SD
      dataFile.println(sdString);
    }



    /* Affichage infos sur LCD (USE_DISPLAY)
        Tout ce qui suit est facultatif
        Différentes infos sont affichées sur le LCD pour notamment permettre de USE_DISPLAYger
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
    //sprintf(lcdString, "GEAR:%16c", gear);
    sprintf(lcdString, "GEAR:%11c/%u", gear, gearValue);
    LCD_PRINT(lcdString);
    LCD_POS(0, ++lcdLine);
    sprintf(lcdString, "BRAKE:%15d", brake);
    LCD_PRINT(lcdString);
    LCD_POS(0, ++lcdLine);
    sprintf(lcdString, "THROTTLE:%12u", throttle);
    LCD_PRINT(lcdString);
    //delay(100);
#endif
  } else {




#ifdef USE_BNO055
    delay(100);
    bno.getCalibration(&cal_sys, &cal_gyro, &cal_accel, &cal_mag); // Etat de calibration capteur 9 axes (facultatif)
    delay(100);
    bno.getSensorOffsets(calibrationData); //> bug bouton power

    //LCD_CLEAR; // Consomme bcp de temps /!\

    // Offsets calibration
    
    lcdLine = 0;
    LCD_POS(0, lcdLine);
    LCD_PRINT("OFFSETS:");
    LCD_POS(0, ++lcdLine);
    sprintf(lcdString, "%5u;%5u;%5u", calibrationData.accel_offset_x, calibrationData.accel_offset_x, calibrationData.accel_offset_z);
    LCD_PRINT(lcdString);
    LCD_POS(0, ++lcdLine);
    sprintf(lcdString, "%5u;%5u;%5u", calibrationData.gyro_offset_x, calibrationData.gyro_offset_y, calibrationData.gyro_offset_z);
    LCD_PRINT(lcdString);
    LCD_POS(0, ++lcdLine);
    sprintf(lcdString, "%5u;%5u;%5u", calibrationData.mag_offset_x, calibrationData.mag_offset_y, calibrationData.mag_offset_z);
    LCD_PRINT(lcdString);
    LCD_POS(0, ++lcdLine);
    sprintf(lcdString, "%5u;%5u", calibrationData.accel_radius, calibrationData.mag_radius);
    LCD_PRINT(lcdString);

    // Etat calibration capteur 9 axes
    LCD_POS(0, ++lcdLine);
    LCD_PRINT("CALIB:");
    sprintf(lcdString, "%d;%d;%d;%d", cal_sys, cal_gyro, cal_accel, cal_mag); // Préparation chaine pour affichage SD/LCD
    LCD_PRINT(lcdString);

    // Etat compteurs bikeRpm/SPEED
    LCD_POS(0, ++lcdLine);
    LCD_PRINT("COMPTEUR:");
    sprintf(lcdString, "%u/%u", bikeRpm, bikeSpeed);
    LCD_PRINT(lcdString);
    delay(100);
#endif
  }
}
