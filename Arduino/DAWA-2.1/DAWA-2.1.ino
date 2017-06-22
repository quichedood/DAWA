/*
 * DAWA 2.1
 * Datalogger moto Triumph
 * Edouard PIGEON - 2017
 */

// A commenter ou non si affichage LCD souhaité ou non
#define USE_DISPLAY

// A commenter ou non si utilisation du capteur de position ou non
#define USE_BNO055

// Librairies
#include <SPI.h>
#include <SD.h>
#include <EEPROMAnything.h>
#ifdef USE_DISPLAY
  //#include <SoftwareSerial.h> // /!\ Commenter vecteurs d'interruption dans SoftwareSerial.cpp (Lignes 234 à 244)
  //#include <LiquidCrystal_I2C.h>
  #define I2C_ADDRESS 0x3C
  #include "SSD1306Ascii.h"
  #include "SSD1306AsciiWire.h"
#endif
#ifdef USE_BNO055
  #include <Adafruit_BNO055.h> // Librairies developpées par ADAFRUIT pour capteur 9 axes "USE_BNO055" (https://github.com/adafruit/Adafruit_USE_BNO055)
  #include <Adafruit_Sensor.h>
#endif
#include <DSRTCLib.h>

#ifdef USE_DISPLAY
  //SoftwareSerial mySerial(3,2); // 2x16 SERIAL // Communication avec le LCD sur port série : pin 2 = TX, pin 3 = RX (non utilisé) / FACULTATIF, utilisé à des fins de USE_DISPLAY
  //LiquidCrystal_I2C lcd(0x3F, 16, 2); // 2x16 I2C // Adresse I2C, colonnes, lignes
  SSD1306AsciiWire oled; // OLED I2C
#endif

#ifdef USE_BNO055
  Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS_B); // USE_BNO055_ADDRESS_B = Adresse 0x29 (COM3 à 1)
#endif

// Commande LCD si USE_DISPLAY activé ou non
#ifdef USE_DISPLAY
 /*#define LCD_PRINT(x)   mySerial.write(x);
 #define LCD_INIT       mySerial.begin(9600);
 #define LCD_POS(x)     posLCD(x); // 128 = ligne 1, 192 = ligne 2
 #define LCD_CLEAR      clearLCD();;*/
 /*#define LCD_PRINT(x)   lcd.print(x);
 #define LCD_INIT       lcd.begin();
 #define LCD_POS(x,y)   lcd.setCursor(x,y); // colonne,ligne
 #define LCD_CLEAR      lcd.clear();*/
 #define LCD_PRINT(x)   oled.print(x); 
 #define LCD_INIT       oled.begin(&Adafruit128x64, I2C_ADDRESS);oled.setFont(System5x7);
 #define LCD_POS(x,y)   oled.setCursor(x, y);
 #define LCD_CLEAR      oled.clear();    
#else
 #define LCD_PRINT(x);
 #define LCD_INIT;
 #define LCD_POS;
 #define LCD_CLEAR;
#endif

// Variable PINs I/O
const int bno_int_pin = 7; // D7
const int timingPin = 16; // A2
const int rpmPin = 5; // D5
const int ledPin = 4; // D4
const int powerPin = 17; //A3
const int brakePin = 6; //D6
const int throttlePin = 1; // A1
const int gearPin = 0; // A0
const uint8_t chipSelect = 8; // CS : port 8

// Variables programme
uint16_t rpm;
char rpmString[6];
char gearvalueString[6];
bool brake = 0;
bool isRunning = 0;
bool int8hz = 0;
uint8_t runCounter = 0, runCalibration = 0;
uint8_t newRun = 0;
uint16_t throttle;
char throttleString[4];
uint16_t gearValue;
char gear = 'N';
char filename[12];
File dataFile;
char sdString[45];
float sec = 0;
char secString[9];
uint8_t gearNCheck = 0;
uint8_t i, count;
DS1339 RTC = DS1339(0, 0);
uint8_t datetime_array[6]={1,1,17,13,37,0}; // 01/01/17 13:37:00
#ifdef USE_BNO055
  char rollString[5];
  char pitchString[5];
  int8_t temperature;
  char temperatureString[4];
  uint8_t cal_sys, cal_gyro, cal_accel, cal_mag = 0;
  char calString[10];
  adafruit_bno055_offsets_t calibrationData;
  //byte *calibrationBuffer = (byte *) &calibrationData;
#endif
#ifdef USE_DISPLAY
  char datetime[11];
#endif

// Fonction utilisée en cas d'erreur (programme arrêté indéfiniement, led clignote rapidement)
void initerror() {
  delay(2000); // Pause 3 sec pour lecture infos LCD
  while(1) {
    digitalWrite(ledPin, digitalRead(ledPin)^1); // Clignotement LED rapide (indique une erreur)
    delay(100);
  }
}

/*
// Efface les 2 lignes du LCD
void clearLCD() {
  LCD_PRINT(254); // Déplacement curseur ...
  LCD_PRINT(128); // ... première ligne
  LCD_PRINT("                "); // Effacement ligne 1
  LCD_PRINT("                "); // Effacement ligne 2 
}

// Positionne le curseur du LCD sur la position "charPosition"
void posLCD(byte charPosition) {
  LCD_PRINT(254); // Déplacement curseur ...
  LCD_PRINT(charPosition); // Ex : 128 = 1ère ligne, 192 = 2ème ligne
}
*/

// Initialisation
void setup() {
  // Init LCD
  LCD_INIT;
  
  // Init I/O
  pinMode(ledPin, OUTPUT); // Sortie LED de contrôle
  pinMode(powerPin, INPUT); // Bouton ON/OFF
  pinMode(brakePin, INPUT); // Entrée capteur frein
  pinMode(throttlePin, INPUT); // Entrée capteur accélerateur
  pinMode(gearPin, INPUT); // Entrée capteur vitesse engagée
  pinMode(rpmPin, INPUT); // Entrée capteur RPM   
  pinMode(timingPin,INPUT);   // Interruption @8Hz provenant du 1339C+CD4060 (port A2)  
  digitalWrite(timingPin, HIGH);  // Activation résistance tirage
  pinMode(bno_int_pin, INPUT); // Interuption générée par le USE_BNO055 (non utilisé)

  // Init + clear LCD (+Backlight)
  delay(1000); // Temporisation pour démarrage LCD
  LCD_CLEAR;
  
  // Init carte SD
  LCD_POS(0,0);
  LCD_PRINT("SD:");
  if (!SD.begin(chipSelect)) {
    LCD_PRINT("FAILED");
    initerror(); 
  }
  LCD_PRINT("OK");  

  // Init capteur 9 axes
  #ifdef USE_BNO055
    LCD_POS(0,1);
    LCD_PRINT("9AXIS:");
    if(!bno.begin()) {
      LCD_PRINT("FAILED");
      //initerror(); 
    }
    LCD_PRINT("OK");
 
    // Utilisateur d'un quartz externe sur capteur 9 axes
    bno.setExtCrystalUse(true);
  #endif
  
  // Initialisation DS1339C (horloge - librairie DSRTCLib)
  RTC.start();
  RTC.setRegister(DSRTCLib_SP, 0x08); // SQW = 4096Hz (divisé par 1024 avec le CD4060 positionné derrière = 4Hz // Attention interruptions sur fronts montants ET descendants donc 8Hz)
  
  // Initialisation timer1 (comptage RPM) 
  noInterrupts();
  TCCR1A = 0b00000000;  // Doc AMTEL p.131
  TCCR1B = 0b01000111;
  TCNT1 = 0; // Init compteur à 0
  interrupts();

  // Initialisation interruptions sur ports analogiques (A1 à A5)
  *digitalPinToPCMSK(timingPin) |= bit (digitalPinToPCMSKbit(timingPin));  // Activation pin A2 pour interruption toutes les 125ms (8Hz)
  PCIFR  |= bit (digitalPinToPCICRbit(timingPin)); // RAZ flags interuption PCIF1
  PCICR  |= bit (digitalPinToPCICRbit(timingPin)); // Activation interruptions pour PCIE1

  /* 
   * D0-D7 = PCINT 16-23 = PCIR2 = PD = PCIE2 = pcmsk2
   * D8-D13 = PCINT 0-5 = PCIR0 = PB = PCIE0 = pcmsk0
   * A0-A5 (D14-D19) = PCINT 8-13 = PCIR1 = PC = PCIE1 = pcmsk1
   */
  
  #ifdef USE_BNO055
    // Temporisation pour lecture des éventuels codes erreur sur LCD
    digitalWrite(ledPin, HIGH); // On allume la LED pour prevenir la possibilité d'effectuer une calibration
    delay(2000); // Pause 2 sec, le temps à l'utilisateur de demander la calibration si souhaité
    LCD_CLEAR;
  
    // Si l'utilisateur appuie sur le bouton, on démarre la calibration
    if (digitalRead(powerPin) == 1) { 
      // Calibration demandée
      LCD_POS(0,0);
      LCD_PRINT("CALIBRATION ..."); 

      // On attend la calibration complète pour mémoriser les offsets !! Les offsets ne sont retournés que si la calibration est complète !!
      while(!bno.isFullyCalibrated()) {
        delay(200); // Pause 0.2 sec
        bno.getCalibration(&cal_sys, &cal_gyro, &cal_accel, &cal_mag); // On récupère l'état de la calibration (de 0 à 3 pour les 3 capteurs + calibration générale)
        sprintf(calString,"%d;%d;%d;%d", cal_sys, cal_gyro, cal_accel, cal_mag); // Mise en forme pour affichage LCD
        LCD_POS(0,1);
        LCD_PRINT(calString); // Affichage de l'état de la calibration sur la 2ème ligne du LCD  
        delay(200); // Pause 0.2 sec
      } 
      
      // Le capteur est calibré, on stocke les bons offsets dans la structure prévue à cet effet (calibrationData)
      bno.getSensorOffsets(calibrationData); 

      // On écrit les 11 offsets sur l'EEPROM
      if(EEPROM_writeAnything(0, calibrationData) == sizeof(adafruit_bno055_offsets_t)) { 
        LCD_CLEAR;
        LCD_POS(0,0);
        LCD_PRINT("CALIB. OK");
      } else {
        initerror();
      }
    } else {
      // Calibration non demandée 
      LCD_POS(0,0); 
      LCD_PRINT("LOAD CALIB. ...");

      // On essaie de charger les 11 valeurs depuis l'EEPROM
      if(EEPROM_readAnything(0, calibrationData) == sizeof(adafruit_bno055_offsets_t)) { 
        bno.setSensorOffsets(calibrationData); // On définit les offsets avec les données récupérées du fichier
        LCD_POS(0,1);
        LCD_PRINT("CALIB. LOADED");  
      } else { // Si aucun fichier de calibration
        LCD_POS(0,1);
        LCD_PRINT("NO CALIB. FILE");
        initerror(); // On considère que c'est une erreur, risque de données faussées
      }
    }
  #endif
  
  delay(1000); // Pause 1 sec pour lecture infos LCD par l'utilisateur
  LCD_CLEAR;

  // Réglage de la date
  dataFile = SD.open("DATE.TXT", FILE_READ); // Si le fichier DATE.TXT est trouvé, il est lu et la nouvelle date est mémorisée  
  if(dataFile) { // Si fichier présent
    LCD_POS(0,0); 
    LCD_PRINT("LOAD DATE ...");
    for (i=0;i<6;i++) { // Le fichier est lu, son contenu est stocké dans la structure
      datetime_array[i] = dataFile.parseInt();
    }
    dataFile.close();
    RTC.setDays(datetime_array[0]);
    RTC.setMonths(datetime_array[1]);
    RTC.setYears(datetime_array[2]);
    RTC.setHours(datetime_array[3]);
    RTC.setMinutes(datetime_array[4]);
    RTC.setSeconds(datetime_array[5]);
    RTC.writeTime();
    
    LCD_POS(0,1);
    LCD_PRINT("DATE SETUP !");  

    // On supprime le fichier contenant la date à définir
    SD.remove("DATE.TXT"); 
    
    delay(1000); // Pause 1 sec pour lecture infos LCD par l'utilisateur
    LCD_CLEAR;
  }
  
  // Setup terminé, on éteint la LED
  digitalWrite(ledPin, LOW);
}

// Interruption PORTS A (Filtre sur port A2) (T=125ms/8Hz)
ISR (PCINT1_vect)
{  
  // Lecture des valeurs au moment de l'interruption
  rpm = ((TCNT1*22)/100)*100; // Lecture registre TCNT1 (compteur du timer1). Aucun débordement possible au vu du signal sorti par le Daytona (75 pulses/125ms @ 1200tr/min ou 5 pulse/100tr/min (100msec))
  TCNT1 = 0; // Réinitialisation du compteur (timer1 / mesure RPM)
  brake = digitalRead(brakePin); // Lecture signal "brakePin" (banchement sur l'alimentation du feu stop avec diviseur de tension pour ramener à 0/+5v)
  throttle = min(((analogRead(throttlePin)/10)*1.23), 100); // Lecture tension port "throttlePin" : la tension varie en fonction de l'ouverture de la poignée de gaz (valeurs brutes mesurées : 137 à 820 / ratio 1.23)
  gearValue = analogRead(gearPin); // Lecture tension port "gearPin" : chaque rapport sort une tension différente (valeurs brutes mesurées : 314, 520, 640, 774, 880, 960, 1023 (N))   
  
  // Appui long pour arrêter/démarrer la mesure
  if (digitalRead(powerPin) == 1) {
    runCounter++;
  } else {
    runCounter = 0;
  }

  // Les calculs et écriture sur carte SD se font en dehors de l'interruption (cf main loop)
  int8hz = 1;
}

// Programme principal
void loop() {   
  // ON/OFF
  if(runCounter >= 10) { // Appui long sur bouton power
    runCounter = 0;
    if(isRunning == 1) { // Si l'enregistrement était en cours ...
      isRunning = 0; // ... on arrête l'enregistrement
      digitalWrite(ledPin, 0); // On éteint la LED indiquant l'aquisition de données en cours
      dataFile.close(); // On ferme le fichier
    } else {
      isRunning = 1; // ... sinon on démarre l'enregistrement
      newRun = 1; // Permet la création d'un nouveau fichier sur la carte SD
    }
  }

  if(isRunning) {
     if(newRun) { // On créé un nouveau fichier et on enregistre          
      newRun = 0;
      RTC.readTime();
      sprintf(filename,"%02u%02u%02u.txt", RTC.getDays(), RTC.getHours(), RTC.getMinutes()); // Limite format nom de fichier "8.3"
      dataFile = SD.open(filename, FILE_WRITE); // Ouverture fichier sur carte SD   
      //if(dataFile) {
        dataFile.println("s;brake;throttle;gear;rpm;roll;pitch;temp"); // Ecriture entêtes fichier
        dataFile.println("   0.000;0;0;N;0;   0;   0;0"); // Ecriture première ligne fichier
        sec = 0; // Reset chrono
      /*} else {
        isRunning = 0;
        digitalWrite(ledPin, 0); // On éteint la LED indiquant l'aquisition de données en cours
      }*/
    }

    // L'interruption vient d'avoir eu lieu, on inscrit les données sur la carte SD après calcul & mise en forme
    if(int8hz == 1) {
      int8hz = 0;
      sec = sec + 0.125; // @8Hz > pas de 0.125s
      digitalWrite(ledPin, digitalRead(ledPin)^1); // Clignotement LED (indicateur bon fonctionnement)

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
      if (gearValue < 360) { // Sélection vitesse en fonction de la tension mesurée sur port analogique
        gear = '1';
      }
      if (gearValue > 500 && gearValue < 540) {
        gear = '2';
      }
      if (gearValue > 620 && gearValue < 660) {
        gear = '3';
      }
      if (gearValue > 754 && gearValue < 794) {
        gear = '4';
      }
      if (gearValue > 860 && gearValue < 900) {
        gear = '5';
      }
      if (gearValue > 940 && gearValue < 980) {
        gear = '6';
      }
      if (gearValue > 1000) { // "N" = 1023
        if(gearNCheck > 3) { // On teste 3 fois de suite pour éviter d'afficher "N" entre 2 passages de vitesse
          gear = 'N';
        } else {
          gearNCheck++;    
        }
      } else {
        gearNCheck = 0;
      }

      // Mise en forme chaines de caractères
      dtostrf(sec, 8, 3, secString); // Préparation chaine pour affichage SD/LCD (conversion float impossible avec sprintf)     
      sprintf(rpmString,"%5u", rpm); // Préparation chaine pour affichage SD/LCD   
      sprintf(gearvalueString,"%5u", gearValue); // Préparation chaine pour affichage SD/LCD (valeurs source gear)
      sprintf(throttleString,"%3u", throttle); // Préparation chaine pour affichage SD/LCD
      #ifdef USE_BNO055
        sprintf(temperatureString,"%d", temperature); // Préparation chaine pour affichage SD/LCD      
        sprintf(sdString,"%s;%d;%u;%c;%u;%s;%s;%s", secString, brake, throttle, gear, rpm, rollString, pitchString, temperatureString); // Préparation chaine pour affichage SD/LCD
      #else
        sprintf(sdString,"%s;%d;%u;%c;%u", secString, brake, throttle, gear, rpm); // Préparation chaine pour affichage SD/LCD
      #endif
      
      // Ecriture carte SD
      dataFile.println(sdString);          
    }

    /* Affichage infos sur LCD (USE_DISPLAY)
     *  Tout ce qui suit est facultatif
     *  Différentes infos sont affichées sur le LCD pour notamment permettre de USE_DISPLAYger
     */
    #ifdef USE_DISPLAY
      LCD_POS(0,0);
      RTC.readTime();
      sprintf(datetime,"%02u/%02u/%u", RTC.getDays(), RTC.getMonths(), RTC.getYears());
      LCD_PRINT(datetime); 
      LCD_POS(72,0);
      sprintf(datetime,"%02u:%02u:%02u", RTC.getHours(), RTC.getMinutes(), RTC.getSeconds());
      LCD_PRINT(datetime); 
      LCD_POS(0,1);
      LCD_PRINT("T:");
      LCD_PRINT(sec); // TIME
      LCD_POS(0,2);
      LCD_PRINT("RPM:");
      LCD_PRINT(rpm); // RPM
      LCD_PRINT("tr/min");
      LCD_POS(0,3);
      LCD_PRINT("SPEED:");
      LCD_PRINT("0"); // SPEED
      LCD_PRINT("km/h");
      LCD_POS(0,4);
      LCD_PRINT("GEAR:");
      LCD_PRINT(gear); // GEAR     
      LCD_POS(0,5);
      LCD_PRINT("BRAKE:");
      if(brake == HIGH) { // BRAKE
        LCD_PRINT("YES");
      } else {
        LCD_PRINT("NO");
      }
      LCD_POS(0,6);
      LCD_PRINT("THROTTLE:");
      LCD_PRINT(throttle); // GEAR      
      LCD_PRINT("%");
      

      // Inclinaison moto
      //LCD_PRINT(rollString);
      //LCD_PRINT(pitchString);

      //delay(100); 
    #endif
  } else {
     LCD_CLEAR; // Consomme bcp de temps /!\
     
    #ifdef USE_BNO055
      bno.getCalibration(&cal_sys, &cal_gyro, &cal_accel, &cal_mag); // Etat de calibration capteur 9 axes (facultatif)
      bno.getSensorOffsets(calibrationData);
      
      // Offsets calibration
      LCD_POS(0,0);
      LCD_PRINT("OFFSETS:");
      LCD_POS(0,1);
      sprintf(calString, "%u;%u;%u", calibrationData.accel_offset_x, calibrationData.accel_offset_x, calibrationData.accel_offset_z); 
      LCD_PRINT(calString);      
      LCD_POS(0,2);   
      sprintf(calString, "%u;%u;%u", calibrationData.gyro_offset_x, calibrationData.gyro_offset_y, calibrationData.gyro_offset_z);  
      LCD_PRINT(calString);
      LCD_POS(0,3);
      sprintf(calString, "%u;%u;%u", calibrationData.mag_offset_x, calibrationData.mag_offset_y, calibrationData.mag_offset_z);  
      LCD_PRINT(calString); 
      LCD_POS(0,4);
      sprintf(calString, "%u;%u", calibrationData.accel_radius, calibrationData.mag_radius); 
      LCD_PRINT(calString);  
      
      // Etat calibration capteur 9 axes 
      LCD_POS(0,5);
      LCD_PRINT("CALIB:");
      sprintf(calString,"%d;%d;%d;%d", cal_sys, cal_gyro, cal_accel, cal_mag); // Préparation chaine pour affichage SD/LCD
      LCD_PRINT(calString); 
    #endif

    delay(500);
  }
}
