/*
 * DAWA 2.1
 * Datalogger moto Triumph
 * Edouard PIGEON - 2017
 */

// Librairies
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h> // /!\ Commenter vecteurs d'interruption dans SoftwareSerial.cpp (Lignes 234 à 244)
#include <Adafruit_BNO055.h> // Librairies developpées par ADAFRUIT pour capteur 9 axes "BNO055" (https://github.com/adafruit/Adafruit_BNO055)
#include <Adafruit_Sensor.h>

SoftwareSerial mySerial(3,2); // Communication avec le LCD sur port série : pin 2 = TX, pin 3 = RX (non utilisé) / FACULTATIF, utilisé à des fins de debug
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS_B); // BNO055_ADDRESS_B = Adresse 0x29 (COM3 à 1)

// Variable PINs I/O
int bno_int_pin = 7; // D7
int timingPin = 16; // A2
int rpmPin = 5; // D5
int ledPin = 4; // D4
int powerPin = 17; //A3
int brakePin = 6; //D6
int throttlePin = 1; // A1
int gearPin = 0; // A0

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
const uint8_t chipSelect = 8; // CS : port 8
char filename[10];
File dataFile;
char sdString[45];
float sec = 0;
char secString[9];
uint8_t gearNCheck = 0;
uint8_t filenameNum = 1;
char filenameBase[] = "log";
char filenameExt[] = ".txt";
char rollString[5];
char pitchString[5];
int8_t temperature;
char temperatureString[4];
uint8_t cal_sys, cal_gyro, cal_accel, cal_mag = 0;
char calString[8]; //10
uint8_t i, count;
adafruit_bno055_offsets_t calibrationData;
byte *calibrationBuffer = (byte *) &calibrationData;

// Fonction utilisée en cas d'erreur (programme arrêté indéfiniement, led clignote rapidement)
void initerror() {
  delay(3000); // Pause 3 sec pour lecture infos LCD
  while(1) {
    digitalWrite(ledPin, digitalRead(ledPin)^1); // Clignotement LED rapide (indique une erreur)
    delay(100);
  }
}

// Efface les 2 lignes du LCD
void clearLCD() {
  mySerial.write(254); // Déplacement curseur ...
  mySerial.write(128); // ... première ligne
  mySerial.write("                "); // Effacement ligne 1
  mySerial.write("                "); // Effacement ligne 2 
}

// Positionne le curseur du LCD sur la position "charPosition"
void posLCD(byte charPosition) {
  mySerial.write(254); // Déplacement curseur ...
  mySerial.write(charPosition); // Ex : 128 = 1ère ligne, 192 = 2ème ligne
}

// Initialisation
void setup() {
  // Init port série 9600 bauds (LCD)
  mySerial.begin(9600);

  // Init I/O
  pinMode(ledPin, OUTPUT); // Sortie LED de contrôle
  pinMode(powerPin, INPUT); // Bouton ON/OFF
  pinMode(brakePin, INPUT); // Entrée capteur frein
  pinMode(throttlePin, INPUT); // Entrée capteur accélerateur
  pinMode(gearPin, INPUT); // Entrée capteur vitesse engagée
  pinMode(rpmPin, INPUT); // Entrée capteur RPM   
  pinMode(timingPin,INPUT);   // Interruption @8Hz provenant du 1339C+CD4060 (port A2)  
  digitalWrite(timingPin, HIGH);  // Activation résistance tirage
  pinMode(bno_int_pin, INPUT); // Interuption générée par le BNO055 (non utilisé)

  // Init + clear LCD (+Backlight)
  //mySerial.write(124); // Backlight (OFF > FULL)
  //mySerial.write(157); // 128, 140, 150, 157 
  delay(1000); // Temporisation pour démarrage LCD
  clearLCD();
  
  // Init carte SD
  posLCD(128);
  mySerial.write("SD:");
  if (!SD.begin(chipSelect)) {
    mySerial.write("FAILED");
    initerror(); 
  }
  mySerial.write("OK");  

  // Init capteur 9 axes
  posLCD(192);
  mySerial.write("9AXIS:");
  if(!bno.begin()) {
    mySerial.write("FAILED");
    initerror(); 
  }
  mySerial.write("OK");
  
  // Utilisateur d'un quartz externe sur capteur 9 axes
  bno.setExtCrystalUse(true);
  
  // Initialisation DS1339C (horloge utilisée comme base de temps)
  Wire.begin();
  Wire.beginTransmission(0x68); // On s'adresse au DS1339C (Envoi adresse chip sur 7 bits : 1101000 soit 0x68)
  Wire.write(byte(0x0E)); // Envoi d'une commande (accès registre 0E)
  Wire.write(byte(0x08)); // SQW = 4096Hz (divisé par 1024 avec le CD4060 positionné derrière = 4Hz // Attention interruptions sur fronts montants ET descendants donc 8Hz)
  Wire.endTransmission();
  
  // Initialisation timer1 (comptage RPM) 
  noInterrupts();
  TCCR1A = 0b00000000;  // Doc AMTEL p.131
  TCCR1B = 0b01000111;
  TCNT1 = 0; // Init compteur à 0
  interrupts();

  // Initialisation interruptions sur ports analogiques (A1 à A5)
  *digitalPinToPCMSK(timingPin) |= bit (digitalPinToPCMSKbit(timingPin));  // Activation pin pour interruption toutes les 125ms (8Hz)
  PCIFR  |= bit (digitalPinToPCICRbit(timingPin)); // RAZ flags interuption PCIF1
  PCICR  |= bit (digitalPinToPCICRbit(timingPin)); // Activation interruptions pour PCIE1

  /* 
   * D0-D7 = PCINT 16-23 = PCIR2 = PD = PCIE2 = pcmsk2
   * D8-D13 = PCINT 0-5 = PCIR0 = PB = PCIE0 = pcmsk0
   * A0-A5 (D14-D19) = PCINT 8-13 = PCIR1 = PC = PCIE1 = pcmsk1
   */
 
  // Temporisation pour lecture des éventuels codes erreur sur LCD
  digitalWrite(ledPin, HIGH); // On allume la LED pour prevenir la possibilité d'effectuer une calibration
  delay(2000); // Pause 2 sec, le temps à l'utilisateur de demander la calibration si souhaité
  clearLCD();

  // Si l'utilisateur appuie sur le bouton, on démarre la calibration
  if (digitalRead(powerPin) == 1) { 
    // Calibration demandée
    posLCD(128);
    mySerial.write("CALIBRATION ...");
    SD.remove("CALIB.TXT"); // On supprime l'ancien fichier de calibration si existant
    dataFile = SD.open("CALIB.TXT", FILE_WRITE); // Création du nouveau fichier de calibration sur carte SD
    if(dataFile) {     
      while(!bno.isFullyCalibrated()) { // On attend la calibration complète pour mémoriser les offsets /!\ Les offsets ne sont retournés que si la calibration est complète /!\
        delay(200); // Pause 0.2 sec
        bno.getCalibration(&cal_sys, &cal_gyro, &cal_accel, &cal_mag); // On récupère l'état de la calibration (de 0 à 3 pour les 3 capteurs + calibration générale)
        sprintf(calString,"%d;%d;%d;%d", cal_sys, cal_gyro, cal_accel, cal_mag); // Mise en forme pour affichage LCD
        posLCD(192);
        mySerial.write(calString); // Affichage de l'état de la calibration sur la 2ème ligne du LCD  
        delay(200); // Pause 0.2 sec    
      }
      bno.getSensorOffsets(calibrationData); // Le capteur est calibré, on stocke les bons offsets dans la structure prévue à cet effet (calibrationData)
      count = dataFile.write(calibrationBuffer, sizeof(adafruit_bno055_offsets_t)); // On écrit le contenu de la structure (offsets) dans le fichier
      if(count == sizeof(adafruit_bno055_offsets_t)) { // Si écriture OK (11 éléments)
        clearLCD();
        posLCD(128);
        mySerial.write("CALIB. OK");
        dataFile.close(); // On ferme le fichier
      } else {
        initerror();  
      }
    } else {
      initerror();      
    }
  } else {
    // Calibration non demandée, on essaie de charger le fichier de calibration si présent sur la carte SD
    dataFile = SD.open("CALIB.TXT", FILE_READ); // Ouverture fichier sur carte SD   
    if(dataFile) { // Si fichier présent
      posLCD(128); 
      mySerial.write("LOAD CALIB. ...");
      for (i=0;i<sizeof(adafruit_bno055_offsets_t);i++) { // Le fichier est lu, son contenu est stocké dans la structure
        *(calibrationBuffer+i) = dataFile.read();
      }
      bno.setSensorOffsets(calibrationData); // On définit les offsets avec les données récupérées du fichier
      posLCD(192);
      mySerial.write("CALIB. LOADED");  
    } else { // Si aucun fichier de calibration
      posLCD(128);
      mySerial.write("NO CALIB. FILE");
      initerror(); // On considère que c'est une erreur, risque de données faussées
    }
    dataFile.close(); // On ferme le fichier
  }
  delay(2000); // Pause 2 sec pour lecture infos LCD par l'utilisateur
  digitalWrite(ledPin, LOW); // Calibration terminée, on éteint la LED
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
      sprintf(filename,"%s%u%s", filenameBase, filenameNum, filenameExt); // Génération nom du fichier
      while(SD.exists(filename)) { // Si le fichier existe déjà on le nomme différement
        filenameNum++;
        sprintf(filename,"%s%u%s", filenameBase, filenameNum, filenameExt); // Génération nom du fichier
      }
      dataFile = SD.open(filename, FILE_WRITE); // Ouverture fichier sur carte SD   
      if(dataFile) {
        dataFile.println("s;brake;throttle;gear;rpm;roll;pitch;temp"); // Ecriture entêtes fichier
        dataFile.println("   0.000;0;0;N;0;   0;   0;0"); // Ecriture première ligne fichier
        sec = 0; // Reset chrono
      } else {
        isRunning = 0;
        digitalWrite(ledPin, 0); // On éteint la LED indiquant l'aquisition de données en cours
      }
    }

    // L'interruption vient d'avoir eu lieu, on inscrit les données sur la carte SD après calcul & mise en forme
    if(int8hz == 1) {
      int8hz = 0;
      sec = sec + 0.125; // @8Hz > pas de 0.125s
      digitalWrite(ledPin, digitalRead(ledPin)^1); // Clignotement LED (indicateur bon fonctionnement)
      
      // Capteur position (roll/pitch) - Précision au degré
      sensors_event_t event; 
      bno.getEvent(&event);
      dtostrf(event.orientation.y, 4, 0, rollString); // Roll
      dtostrf(event.orientation.z, 4, 0, pitchString); // Pitch
      temperature = bno.getTemp(); // Température ambiante
      bno.getCalibration(&cal_sys, &cal_gyro, &cal_accel, &cal_mag); // Etat de calibration capteur 9 axes (facultatif)
      
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
      sprintf(temperatureString,"%d", temperature); // Préparation chaine pour affichage SD/LCD
      sprintf(sdString,"%s;%d;%u;%c;%u;%s;%s;%s", secString, brake, throttle, gear, rpm, rollString, pitchString, temperatureString); // Préparation chaine pour affichage SD/LCD
      
      // Ecriture carte SD
      dataFile.println(sdString);          
    }

    // Affichage infos sur LCD (DEBUG)
    clearLCD();
    posLCD(128);
    //sprintf(caloffsetString, "%u;%u;%u", calibrationData.accel_offset_x, calibrationData.gyro_offset_y, calibrationData.mag_offset_z);    
    //mySerial.write(throttleString); // Position gaz
    mySerial.write(secString); // Chrono
    //mySerial.write(rpmString); // RPM
    mySerial.write('/');
    sprintf(calString,"%d;%d;%d;%d", cal_sys, cal_gyro, cal_accel, cal_mag); // Préparation chaine pour affichage SD/LCD
    mySerial.write(calString); // Etat calibration capteur 9 axes
    posLCD(192);
    mySerial.write(rollString);
    mySerial.write('/');
    mySerial.write(pitchString);
    //mySerial.write(gear); // Rapport engagé
    //if(brake == HIGH) { // Freinage
    //  mySerial.write("B");
    //} else {
    //  mySerial.write("-");
    //}
  } else {
    clearLCD();
    posLCD(128);
    mySerial.write("READY !");
    posLCD(192);
    mySerial.write("PRESS TO START");
    delay(100);  
  }
}
