/**************************************************************
  #fGetLastRun > Print last session
**************************************************************/
void fGetHelp() {
  BLUETOOTH_PORT.println(F("Available commands"));
  BLUETOOTH_PORT.println(F(">>> General"));
  BLUETOOTH_PORT.println(F("1-Show realtime data"));
  BLUETOOTH_PORT.println(F("2-Show last run"));
  BLUETOOTH_PORT.println(F("3-Show today runs"));
  BLUETOOTH_PORT.println(F("4-Show best laps on this track"));
  BLUETOOTH_PORT.println(F(">>> Calibration"));
  BLUETOOTH_PORT.println(F("20-Calibrate throttle"));
  BLUETOOTH_PORT.println(F("21-Calibrate gears"));
  BLUETOOTH_PORT.println(F("22-Calibrate 9 axis sensor"));
  BLUETOOTH_PORT.println(F(">>> Analog sensors"));
  BLUETOOTH_PORT.println(F("30-Show analog ports state"));
  BLUETOOTH_PORT.println(F("31-Toggle port 1"));
  BLUETOOTH_PORT.println(F("32-Toggle port 2"));
  BLUETOOTH_PORT.println(F("33-Toggle port 3"));
  BLUETOOTH_PORT.println(F("34-Toggle port 4"));
  BLUETOOTH_PORT.println(F("35-Toggle port 5"));
  BLUETOOTH_PORT.println(F("36-Toggle port 6"));
  BLUETOOTH_PORT.println(F("37-Toggle port GEAR"));
  BLUETOOTH_PORT.println(F("38-Toggle port THROTTLE"));
  BLUETOOTH_PORT.println(F(">>> Infrared temp. sensors"));
  BLUETOOTH_PORT.println(F("40-Show defined IR sensors"));
  BLUETOOTH_PORT.println(F("41-Reset defined IR sensors"));
  BLUETOOTH_PORT.println(F("42-Add new IR sensor (unplug others)"));
}

/**************************************************************
  #fGetLastRun > Print last session
**************************************************************/
void fGetLastRun() {
  char trackName[16], lapTime[10], bestLapTimeStr[10], histDate[14];
  float lapTimeSec, bestLapTime;
  uint16_t bestLapId;
  int32_t histTimestamp;

  historyFile = sd.open("HISTORY.csv", FILE_READ); // Read entire history file to get last session laptime filename (based on timestamp)
  if (historyFile) {
    while (historyFile.available()) {
      csvReadInt32(&historyFile, &histTimestamp, csvDelim);
      csvReadText(&historyFile, histDate, sizeof(histDate), csvDelim);
      csvReadUint16(&historyFile, &histTrackId, csvDelim);
    }
    historyFile.close();

    printDateTime(histTimestamp, BLUETOOTH_PORT);

    trackFile = sd.open("TRACKS.csv", FILE_READ); // Read trackfile to get track name
    if (trackFile) {
      while (trackFile.available()) {
        csvReadUint16(&trackFile, &trackId, csvDelim);
        csvReadText(&trackFile, trackName, sizeof(trackName), csvDelim);
        csvReadInt32(&trackFile, &flineLat1, csvDelim);
        csvReadInt32(&trackFile, &flineLon1, csvDelim);
        csvReadInt32(&trackFile, &flineLat2, csvDelim);
        csvReadInt32(&trackFile, &flineLon2, csvDelim);
        if (trackId == histTrackId) {
          trackFile.close();
          BLUETOOTH_PORT.print(F("Track : "));
          BLUETOOTH_PORT.println(trackName);
          break;
        }
      }
      trackFile.close();
    }

    sprintf(filename, "%s-LAPTIMES.csv", histDate);
    lapFile = sd.open(filename, FILE_READ); // Read last session laptime filename
    if (lapFile && lapFile.size() > 0) {
      bestLapTime = 9999.00;
      while (lapFile.available()) {
        csvReadUint16(&lapFile, &lapId, csvDelim);
        csvReadText(&lapFile, lapTime, sizeof(lapTime), csvDelim);
        csvReadFloat(&lapFile, &lapTimeSec, csvDelim);
        if (lapTimeSec < bestLapTime) {
          bestLapTime = lapTimeSec;
          bestLapId = lapId;
          strcpy(bestLapTimeStr, lapTime);
        }
        BLUETOOTH_PORT.print(F("L"));
        BLUETOOTH_PORT.print(lapId);
        BLUETOOTH_PORT.print(F(" > "));
        BLUETOOTH_PORT.println(lapTime);
      }
      BLUETOOTH_PORT.print(F("Best : "));
      BLUETOOTH_PORT.print(F("L"));
      BLUETOOTH_PORT.print(bestLapId);
      BLUETOOTH_PORT.print(F(" > "));
      BLUETOOTH_PORT.println(bestLapTimeStr);
      lapFile.close();
    } else {
      BLUETOOTH_PORT.println(F("No valid lap"));
    }
  }
}

/**************************************************************
  #fGetToday > Print sessions in the last 12h
**************************************************************/
void fGetToday() {
  char trackName[16], lapTime[10], histDate[14];
  int32_t histTimestamp;
  float lapTimeSec, bestLapTime;

  historyFile = sd.open("HISTORY.csv", FILE_READ);
  if (historyFile) {
    BLUETOOTH_PORT.println(F("----------------------------------"));
    while (historyFile.available()) {
      csvReadInt32(&historyFile, &histTimestamp, csvDelim);
      csvReadText(&historyFile, histDate, sizeof(histDate), csvDelim);
      csvReadUint16(&historyFile, &trackId, csvDelim);
      printDateTime(histTimestamp, BLUETOOTH_PORT);
      if (histTimestamp > (fix_data.dateTime - (12 * 3600))) {
        trackFile = sd.open("TRACKS.csv", FILE_READ); // Read trackfile to get track name
        if (trackFile) {
          while (trackFile.available()) {
            csvReadUint16(&trackFile, &trackId, csvDelim);
            csvReadText(&trackFile, trackName, sizeof(trackName), csvDelim);
            csvReadInt32(&trackFile, &flineLat1, csvDelim);
            csvReadInt32(&trackFile, &flineLon1, csvDelim);
            csvReadInt32(&trackFile, &flineLat2, csvDelim);
            csvReadInt32(&trackFile, &flineLon2, csvDelim);

            if (trackId == histTrackId) {
              trackFile.close();
              BLUETOOTH_PORT.print(F("Track : "));
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
            csvReadUint16(&lapFile, &lapId, csvDelim);
            csvReadText(&lapFile, lapTime, sizeof(lapTime), csvDelim);
            csvReadFloat(&lapFile, &lapTimeSec, csvDelim);
            if (lapTimeSec < bestLapTime) {
              bestLapTime = lapTimeSec;
            }
            BLUETOOTH_PORT.print(F("L"));
            BLUETOOTH_PORT.print(lapId);
            BLUETOOTH_PORT.print(F(" > "));
            BLUETOOTH_PORT.println(lapTime);
          }
          BLUETOOTH_PORT.print(F("Best : "));
          BLUETOOTH_PORT.println(bestLapTime);
          BLUETOOTH_PORT.println(F("----------------------------------"));
          lapFile.close();
        }
      }
    }
    historyFile.close();
  }
}

/**************************************************************
  #fGetTrackBest > Print best laptimes
**************************************************************/
void fGetTrackBest() {
  // TODO
}

/**************************************************************
  #fGetData > Print realtime data (like on the oled screen)
**************************************************************/
void fGetData() {
  printData1(fix_data, BLUETOOTH_PORT);
  printData2(fix_data, BLUETOOTH_PORT);
}

/**************************************************************
  #fCalThrottle > Calibrate throttle (max value)
**************************************************************/
void fCalThrottle() {
  BLUETOOTH_PORT.print(F("Set THROTTLE to 100% ... "));
  delay(2000);
  anaValues[6] = readAdcValue(0x26);
  BLUETOOTH_PORT.println(anaValues[6]);
  if (EEPROM_writeAnything(31, anaValues[6]) == sizeof(anaValues[6])) {
    throttleMax = anaValues[6];
    BLUETOOTH_PORT.println(F("Throttle calibration done !"));
  } else {
    BLUETOOTH_PORT.println(F("Throttle calibration failed !"));
  }
}

/**************************************************************
  #fCalGear > Calibrate gear (values for each gear)
**************************************************************/
void fCalGear() {
  for (uint8_t i = 0; i < 7; i++) {
    if (i == 0) {
      BLUETOOTH_PORT.print(F("Set GEAR to N"));
    } else {
      BLUETOOTH_PORT.print(F("Set GEAR to "));
      BLUETOOTH_PORT.print(i);
    }
    BLUETOOTH_PORT.print(F(" ... "));
    delay(3000);
    gearValues[i] = readAdcValue(0x27);
    BLUETOOTH_PORT.println(gearValues[i]);
  }
  if (EEPROM_writeAnything(33, gearValues) == sizeof(gearValues)) {
    BLUETOOTH_PORT.println(F("Gear calibration done !"));
  } else {
    BLUETOOTH_PORT.println(F("Gear calibration failed !"));
  }
}

/**************************************************************
  #fCal9axis > Calibrate 9-axis sensor
**************************************************************/
void fCal9axis() {
  OLED_PORT.clear();
  OLED_PORT.setCursor(0, 0); // Set cursor upper-left
  OLED_PORT.println(F("9-Axis calibration"));
  OLED_PORT.println(F("Move DAWA by doing 8"));
  OLED_PORT.println(F("Place DAWA in 6 pos."));
  OLED_PORT.println(F("+X,-X,+Y,-Y,+Z,-Z"));
  BLUETOOTH_PORT.println(F("9-Axis calibration"));
  BLUETOOTH_PORT.println(F("1) Move DAWA by doing eights"));
  BLUETOOTH_PORT.println(F("2) Place DAWA in 6 standing positions for +X, -X, +Y, -Y, +Z and -Z"));
  while (!bno.isFullyCalibrated()) { // We wait until sensor fully calibrated !! Offsets are returned only if sensor is calibrated !!
    delay(200);
    bno.getCalibration(&calSys, &calGyro, &calAccel, &calMag); // Get calibration state (from 0 to 3 for the 3 sensors + main sensor calibration)
    OLED_PORT.setCursor(42, 5);
    OLED_PORT.print(calSys);
    OLED_PORT.print(F(";"));
    OLED_PORT.print(calGyro);
    OLED_PORT.print(F(";"));
    OLED_PORT.print(calAccel);
    OLED_PORT.print(F(";"));
    OLED_PORT.print(calMag);
    delay(200);
  }
  bno.getSensorOffsets(calibrationData); // Sensor is calibrated, we save offsets to var (calibrationData)
  OLED_PORT.setCursor(36, 7);
  if (EEPROM_writeAnything(0, calibrationData) == sizeof(adafruit_bno055_offsets_t)) { // We save the 11 offsets to eeprom
    OLED_PORT.println(F("OK"));
    BLUETOOTH_PORT.println(F("9-axis calibration done !"));
  } else {
    OLED_PORT.println(F("FAILED"));
    BLUETOOTH_PORT.println(F("9-axis calibration failed !"));
  }
  delay(1000);
  OLED_PORT.clear();
}
/**************************************************************
  #fTogAna1 > Toggle analogic port 1
**************************************************************/
void fTogAna1() {
  anaDisabledBits ^= B00000001;
  if (EEPROM_writeAnything(30, anaDisabledBits) == sizeof(anaDisabledBits)) {
    tmpComp = anaDisabledBits & B00000001;
    if (tmpComp == B00000001) {
      BLUETOOTH_PORT.println(F("ADC port 1 DISABLED, please restart"));
    } else {
      BLUETOOTH_PORT.println(F("ADC port 1 ENABLED, please restart"));
    }
  } else {
    BLUETOOTH_PORT.println(F("Error : port not toggled !"));
  }
}

/**************************************************************
  #fTogAna2 > Toggle analogic port 2
**************************************************************/
void fTogAna2() {
  anaDisabledBits ^= B00000010;
  if (EEPROM_writeAnything(30, anaDisabledBits) == sizeof(anaDisabledBits)) {
    tmpComp = anaDisabledBits & B00000010;
    if (tmpComp == B00000010) {
      BLUETOOTH_PORT.println(F("ADC port 2 DISABLED, please restart"));
    } else {
      BLUETOOTH_PORT.println(F("ADC port 2 ENABLED, please restart"));
    }
  } else {
    BLUETOOTH_PORT.println(F("Error : port not toggled !"));
  }
}

/**************************************************************
  #fTogAna3 > Toggle analogic port 3
**************************************************************/
void fTogAna3() {
  anaDisabledBits ^= B00000100;
  if (EEPROM_writeAnything(30, anaDisabledBits) == sizeof(anaDisabledBits)) {
    tmpComp = anaDisabledBits & B00000100;
    if (tmpComp == B00000100) {
      BLUETOOTH_PORT.println(F("ADC port 3 DISABLED, please restart"));
    } else {
      BLUETOOTH_PORT.println(F("ADC port 3 ENABLED, please restart"));
    }
  } else {
    BLUETOOTH_PORT.println(F("Error : port not toggled !"));
  }
}

/**************************************************************
  #fTogAna4 > Toggle analogic port 4
**************************************************************/
void fTogAna4() {
  anaDisabledBits ^= B00001000;
  if (EEPROM_writeAnything(30, anaDisabledBits) == sizeof(anaDisabledBits)) {
    tmpComp = anaDisabledBits & B00001000;
    if (tmpComp == B00001000) {
      BLUETOOTH_PORT.println(F("ADC port 4 DISABLED, please restart"));
    } else {
      BLUETOOTH_PORT.println(F("ADC port 4 ENABLED, please restart"));
    }
  } else {
    BLUETOOTH_PORT.println(F("Error : port not toggled !"));
  }
}

/**************************************************************
  #fTogAna5 > Toggle analogic port 5
**************************************************************/
void fTogAna5() {
  anaDisabledBits ^= B00010000;
  if (EEPROM_writeAnything(30, anaDisabledBits) == sizeof(anaDisabledBits)) {
    tmpComp = anaDisabledBits & B00010000;
    if (tmpComp == B00010000) {
      BLUETOOTH_PORT.println(F("ADC port 5 DISABLED, please restart"));
    } else {
      BLUETOOTH_PORT.println(F("ADC port 5 ENABLED, please restart"));
    }
  } else {
    BLUETOOTH_PORT.println(F("Error : port not toggled !"));
  }
}

/**************************************************************
  #fTogAna6 > Toggle analogic port 6
**************************************************************/
void fTogAna6() {
  anaDisabledBits ^= B00100000;
  if (EEPROM_writeAnything(30, anaDisabledBits) == sizeof(anaDisabledBits)) {
    tmpComp = anaDisabledBits & B00100000;
    if (tmpComp == B00100000) {
      BLUETOOTH_PORT.println(F("ADC port 6 DISABLED, please restart"));
    } else {
      BLUETOOTH_PORT.println(F("ADC port 6 ENABLED, please restart"));
    }
  } else {
    BLUETOOTH_PORT.println(F("Error : port not toggled !"));
  }
}

/**************************************************************
  #fTogAna7 > Toggle analogic port 7
**************************************************************/
void fTogAna7() {
  anaDisabledBits ^= B01000000;
  if (EEPROM_writeAnything(30, anaDisabledBits) == sizeof(anaDisabledBits)) {
    tmpComp = anaDisabledBits & B01000000;
    if (tmpComp == B01000000) {
      BLUETOOTH_PORT.println(F("ADC port THROTTLE DISABLED, please restart"));
    } else {
      BLUETOOTH_PORT.println(F("ADC port THROTTLE ENABLED, please restart"));
    }
  } else {
    BLUETOOTH_PORT.println(F("Error : port not toggled !"));
  }
}

/**************************************************************
  #fTogAna8 > Toggle analogic port 8
**************************************************************/
void fTogAna8() {
  anaDisabledBits ^= B10000000;
  if (EEPROM_writeAnything(30, anaDisabledBits) == sizeof(anaDisabledBits)) {
    tmpComp = anaDisabledBits & B10000000;
    if (tmpComp == B10000000) {
      BLUETOOTH_PORT.println(F("ADC port GEAR DISABLED, please restart"));
    } else {
      BLUETOOTH_PORT.println(F("ADC port GEAR ENABLED, please restart"));
    }
  } else {
    BLUETOOTH_PORT.println(F("Error : port not toggled !"));
  }
}

/**************************************************************
  #fShowADC > Show ADC ports state
**************************************************************/
void fShowADC() {
  showADCPortState(BLUETOOTH_PORT);
}

/**************************************************************
  #fClearMlxAddr > Clear saved I2C address for all MLX declared chip
**************************************************************/
void fClearMlxAddr() {
  for (uint8_t i = 0; i < maxMlx; i++) {
    mlxAddresses[i] = 0x00;
  }
  if (EEPROM_writeAnything(50, mlxAddresses) == sizeof(mlxAddresses)) {
    BLUETOOTH_PORT.println(F("MLX I2C addresses cleared"));
  } else {
    BLUETOOTH_PORT.println(F("Error clearing MLX I2C addresses"));
  }
}

/**************************************************************
  #fListMlxAddr > List saved I2C address for all MLX declared chip
**************************************************************/
void fListMlxAddr() {
  for (uint8_t i = 0; i < maxMlx; i++) {
    BLUETOOTH_PORT.print(F("MLX I2C address : "));
    BLUETOOTH_PORT.println(mlxAddresses[i], HEX);
  }
}

/**************************************************************
  #fAddMlx > Add a new MLX chip
**************************************************************/
void fAddMlx() {
  bool newMlx = false;
  uint8_t mlxAddress;
  MLX90614 mynewmlx = MLX90614(0); // 0 = broadcast, only one MLX chip shloud be plugged on I2C bus
  mlxAddress = (uint8_t)mynewmlx.readEEProm(mlxEepAddr);
  BLUETOOTH_PORT.print(F("New MLX found : "));
  BLUETOOTH_PORT.println(mlxAddress, HEX);

  for (uint8_t i = 0; i < maxMlx; i++) {
    if (mlxAddresses[i] == 0x00) { // First available address is used (default EEPROM content is 0x00)
      mlxAddresses[i] = firstMlxAddress + i;
      mynewmlx.writeEEProm(mlxEepAddr, mlxAddresses[i]);
      BLUETOOTH_PORT.print(F("Change MLX address to : "));
      BLUETOOTH_PORT.println(mlxAddresses[i], HEX);
      if (EEPROM_writeAnything(50, mlxAddresses) == sizeof(mlxAddresses)) {
        BLUETOOTH_PORT.print(F("MLX sensor added, new address : "));
        BLUETOOTH_PORT.println((uint8_t)mynewmlx.readEEProm(mlxEepAddr), HEX);
        newMlx = true;
      } else {
        BLUETOOTH_PORT.println(F("Error adding new MLX sensor"));
      }
      break;
    }
  }
  if (!newMlx) {
    BLUETOOTH_PORT.println(F("No more MLX I2C addresses available or MLX EEPROM corrupt"));
  }
}


/**************************************************************
  SERCOM5_Handler : Secondary hardware serial for bluetooth
**************************************************************/
void SERCOM5_Handler() {
  BLUETOOTH_PORT.IrqHandler();
}

/**************************************************************
  Output DATE+TIME based on a timestamp
**************************************************************/
void printDateTime(uint32_t timestamp, Print &OUT_PORT) {
  if (day(timestamp) < 10) OUT_PORT.print(F("0")); // Leading zeros
  OUT_PORT.print(day(timestamp));
  OUT_PORT.print(F("/"));
  if (month(timestamp) < 10) OUT_PORT.print(F("0")); // Leading zeros
  OUT_PORT.print(month(timestamp));
  OUT_PORT.print(F("/"));
  OUT_PORT.print(year(timestamp));
  OUT_PORT.print(F(" "));
  if (hour(timestamp) < 10) OUT_PORT.print(F("0")); // Leading zeros
  OUT_PORT.print(hour(timestamp));
  OUT_PORT.print(F(":"));
  if (minute(timestamp) < 10) OUT_PORT.print(F("0")); // Leading zeros
  OUT_PORT.print(minute(timestamp));
  OUT_PORT.print(F(":"));
  if (second(timestamp) < 10) OUT_PORT.print(F("0")); // Leading zeros
  OUT_PORT.println(second(timestamp));
}

/**************************************************************
  #processCharInput > Build a string character by character (serial bluetooth)
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
  #initError > Triggered when initialization error (program stop and slow blinking led)
**************************************************************/
void initError() {
  delay(2000); // Pause 3 sec pour lecture infos LCD
  while (1) {
    digitalWrite(ledPin, digitalRead(ledPin) ^ 1); // Quick LED blink (there's an error)
    delay(200);
  }
}

/**************************************************************
  #sendUBX > Send UBX commands to UBLOX GPS
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
int csvReadText(File * file, char* str, size_t size, char delim) {
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

int csvReadInt32(File * file, int32_t* num, char delim) {
  char buf[20];
  char* ptr;
  int rtn = csvReadText(file, buf, sizeof(buf), delim);
  if (rtn < 0) return rtn;
  *num = strtol(buf, &ptr, 10);
  if (buf == ptr) return -3;
  while (isspace(*ptr)) ptr++;
  return *ptr == 0 ? rtn : -4;
}

int csvReadInt16(File * file, int16_t* num, char delim) {
  int32_t tmp;
  int rtn = csvReadInt32(file, &tmp, delim);
  if (rtn < 0) return rtn;
  if (tmp < INT_MIN || tmp > INT_MAX) return -5;
  *num = tmp;
  return rtn;
}

int csvReadUint32(File * file, uint32_t* num, char delim) {
  char buf[20];
  char* ptr;
  int rtn = csvReadText(file, buf, sizeof(buf), delim);
  if (rtn < 0) return rtn;
  *num = strtoul(buf, &ptr, 10);
  if (buf == ptr) return -3;
  while (isspace(*ptr)) ptr++;
  return *ptr == 0 ? rtn : -4;
}

int csvReadUint16(File * file, uint16_t* num, char delim) {
  uint32_t tmp;
  int rtn = csvReadUint32(file, &tmp, delim);
  if (rtn < 0) return rtn;
  if (tmp > UINT_MAX) return -5;
  *num = tmp;
  return rtn;
}

int csvReadDouble(File * file, double * num, char delim) {
  char buf[20];
  char* ptr;
  int rtn = csvReadText(file, buf, sizeof(buf), delim);
  if (rtn < 0) return rtn;
  *num = strtod(buf, &ptr);
  if (buf == ptr) return -3;
  while (isspace(*ptr)) ptr++;
  return *ptr == 0 ? rtn : -4;
}

int csvReadFloat(File * file, float * num, char delim) {
  double tmp;
  int rtn = csvReadDouble(file, &tmp, delim);
  if (rtn < 0)return rtn;
  // could test for too large.
  *num = tmp;
  return rtn;
}

/**************************************************************
  #segIntersect > Calculate 2 line segments intersection
**************************************************************/
bool segIntersect(int32_t pos_now_lat, int32_t pos_now_lon, int32_t pos_prev_lat, int32_t pos_prev_lon, int32_t trackLat1, int32_t trackLon1, int32_t trackLat2, int32_t trackLon2, int32_t & pos_cross_lat, int32_t & pos_cross_lon) {
  bool denomPositive;
  float denom, s_numer, t_numer, t;
  int32_t track_pos_x, track_pos_y, pos_x, pos_y, trackLon, trackLat;

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
  #gpsDistance > Calculate distance between 2 GPS coords (unit = meters) - haversine formula
**************************************************************/
float gpsDistance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2) {
  float dlam, dphi, p = 0.017453292519943295; // (Pi / 180)
  dphi = p * (lat1 + lat2) * 0.5e-7; //average latitude in radians
  float cphi = cos(dphi);
  dphi = p * ( lat2 - lat1) * 1.0e-7; //differences in degrees (to radians)
  dlam = p * ( lon2 - lon1) * 1.0e-7;
  dlam *= cphi;  //correct for latitude
  return 6371000.0 * sqrt(dphi * dphi + dlam * dlam);
}

/**************************************************************
  #timeAdd > Add a time [arg0] (Ex : 5.14) to another which is composed of seconds [arg1] and milliseconds [arg2]. Return seconds [arg3] and milliseconds [arg4]
**************************************************************/
void timeAdd(float timeSecCsec, int32_t endSec, int32_t endCsec, int32_t &returnSec, int32_t &returnCsec) {
  returnSec = endSec + (int32_t)(timeSecCsec + (endCsec / 100.00));
  returnCsec = ((timeSecCsec + (endCsec / 100.00)) - (int32_t)(timeSecCsec + (endCsec / 100.00))) * 100;
}

/**************************************************************
  #timeSubstract > Substract a time [arg0][arg1] to another [arg2][arg3]. Return seconds [arg4] and milliseconds [arg6]
**************************************************************/
void timeSubstract(int32_t s1, int32_t cs1, int32_t s2, int32_t cs2, int32_t &returnSec, int32_t &returnCsec) {
  returnCsec = cs1 - cs2;
  if (returnCsec < 0) {
    returnSec = (s1 - s2) - 1;
    returnCsec += 100;
  } else {
    returnSec = s1 - s2;
  }
}

/**************************************************************
  #printData1 > Print realtime data on one output (oled screen or bluetooth serial)
**************************************************************/
void printData1(gps_fix & fix_data, Print & OUT_PORT) {
  if (fix_data.valid.date && fix_data.valid.time) {
    if (fix_data.dateTime.date < 10) OUT_PORT.print(F("0")); // Leading zeros
    OUT_PORT.print(fix_data.dateTime.date);
    OUT_PORT.print(F("/"));
    if (fix_data.dateTime.month < 10) OUT_PORT.print(F("0")); // Leading zeros
    OUT_PORT.print(fix_data.dateTime.month);
    OUT_PORT.print(F("/"));
    if (fix_data.dateTime.year < 10) OUT_PORT.print(F("0")); // Leading zeros
    OUT_PORT.print(fix_data.dateTime.year);
    OUT_PORT.print(F(" "));
    if (fix_data.dateTime.hours < 10) OUT_PORT.print(F("0")); // Leading zeros
    OUT_PORT.print(fix_data.dateTime.hours);
    OUT_PORT.print(F(":"));
    if (fix_data.dateTime.minutes < 10) OUT_PORT.print(F("0")); // Leading zeros
    OUT_PORT.print(fix_data.dateTime.minutes);
    OUT_PORT.print(F(":"));
    if (fix_data.dateTime.seconds < 10) OUT_PORT.print(F("0")); // Leading zeros
    OUT_PORT.println(fix_data.dateTime.seconds);
  } else {
    OUT_PORT.println(F("No GPS signal"));
  }
  OUT_PORT.print(F("GPS:"));
  OUT_PORT.print(fix_data.dateTime);
  OUT_PORT.print(F("."));
  if (fix_data.dateTime_cs < 10) OUT_PORT.print(F("0")); // Leading zeros (remember "fix_data.dateTime_cs" is an integer !!)
  OUT_PORT.println(fix_data.dateTime_cs);
  OUT_PORT.print(F("RPM:       "));
  OUT_PORT.println(bikeRpm);
  OUT_PORT.print(F("SPEED:     "));
  OUT_PORT.println(bikeSpeed);
  OUT_PORT.print(F("GEAR:      "));
  OUT_PORT.println(gear);
  OUT_PORT.print(F("BRAKE:     "));
  OUT_PORT.println(brake);
  OUT_PORT.print(F("THROTTLE:"));
  if (throttle < 10) OUT_PORT.print(F("00"));
  if (throttle >= 10 && throttle < 100) OUT_PORT.print(F("0"));
  OUT_PORT.println(throttle);
}

/**************************************************************
  #printData2 > Print realtime data on one output (oled screen or bluetooth serial)
**************************************************************/
void printData2(gps_fix & fix_data, Print & OUT_PORT) {
  uint8_t bitShift = B00000001, tmpComp;
  uint8_t printedValues = 0;
  if (fix_data.valid.date && fix_data.valid.time) {
    if (fix_data.dateTime.date < 10) OUT_PORT.print(F("0")); // Leading zeros
    OUT_PORT.print(fix_data.dateTime.date);
    OUT_PORT.print(F("/"));
    if (fix_data.dateTime.month < 10) OUT_PORT.print(F("0")); // Leading zeros
    OUT_PORT.print(fix_data.dateTime.month);
    OUT_PORT.print(F("/"));
    if (fix_data.dateTime.year < 10) OUT_PORT.print(F("0")); // Leading zeros
    OUT_PORT.print(fix_data.dateTime.year);
    OUT_PORT.print(F(" "));
    if (fix_data.dateTime.hours < 10) OUT_PORT.print(F("0")); // Leading zeros
    OUT_PORT.print(fix_data.dateTime.hours);
    OUT_PORT.print(F(":"));
    if (fix_data.dateTime.minutes < 10) OUT_PORT.print(F("0")); // Leading zeros
    OUT_PORT.print(fix_data.dateTime.minutes);
    OUT_PORT.print(F(":"));
    if (fix_data.dateTime.seconds < 10) OUT_PORT.print(F("0")); // Leading zeros
    OUT_PORT.println(fix_data.dateTime.seconds);
  } else {
    OUT_PORT.println(F("No GPS signal"));
  }
  OUT_PORT.print(F("CALIB:"));
  OUT_PORT.print(calSys);
  OUT_PORT.print(F(";"));
  OUT_PORT.print(calGyro);
  OUT_PORT.print(F(";"));
  OUT_PORT.print(calAccel);
  OUT_PORT.print(F(";"));
  OUT_PORT.println(calMag);
  for (uint8_t i = 0; i < 8; i++) { // We parse the 8 analog values
    tmpComp = bitShift & anaDisabledBits;
    if (i == 6 && printedValues > 0) {
      OUT_PORT.println();
    }
    if (tmpComp == bitShift) { // If analog is disabled ...
      if (i == 6) {
        OUT_PORT.println(F("THROTTLE:N/C"));
      } else if (i == 7) {
        OUT_PORT.println(F("GEAR:N/C"));
      }
    } else { // If analog port is enabled ...
      if (i == 6) { // If "Throttle" ...
        OUT_PORT.print(F("THROTTLE:"));
        if (throttle < 10) OUT_PORT.print(F("00"));
        if (throttle >= 10 && throttle < 100) OUT_PORT.print(F("0"));
        OUT_PORT.println(throttle);
      } else if (i == 7) { // If "Gear" ...
        OUT_PORT.print(F("GEAR:"));
        OUT_PORT.println(gear);
      } else { // If other analog ports ...
        if (printedValues % 3 == 0) {
          if (printedValues > 0) {
            OUT_PORT.println();
          }
          OUT_PORT.print(F("ADC:"));
        }
        if (anaValues[i] < 10) OUT_PORT.print(F("000"));
        if (anaValues[i] >= 10 && anaValues[i] < 100) OUT_PORT.print(F("00"));
        if (anaValues[i] >= 100 && anaValues[i] < 1000) OUT_PORT.print(F("0"));
        OUT_PORT.print(anaValues[i]);
        OUT_PORT.print(F(" "));
        printedValues++;
      }
    }
    bitShift = bitShift << 1;
  }
  OUT_PORT.print(F("AMB. T:"));
  OUT_PORT.println(temperature);
  OUT_PORT.print(F("MLX T:"));
  for (uint8_t i = 0; i < maxMlx; i++) {
    if (mlxAddresses[i] != 0x00) {
      OUT_PORT.print(mlxValues[i]);
      OUT_PORT.print(F(" "));
    }
  }
}

/**************************************************************
  #initADC > Initialize ADC (I2C)
**************************************************************/
uint8_t initADC() {
  Wire.beginTransmission(0x1D);
  Wire.write(uint8_t(0x0B)); // Advanced Configuration Register
  Wire.write(uint8_t(0x03)); // 00000011 (Vref ext & mode = 1)
  Wire.endTransmission();

  Wire.beginTransmission(0x1D);
  Wire.write(uint8_t(0x07)); // Conversion Rate Register
  Wire.write(uint8_t(0x01)); // 00000001 (Conversion mode = continuous)
  Wire.endTransmission();

  Wire.beginTransmission(0x1D);
  Wire.write(uint8_t(0x08)); // Channel Disable Register
  Wire.write(uint8_t(anaDisabledBits)); // Enable or disable channels
  Wire.endTransmission();

  Wire.beginTransmission(0x1D);
  Wire.write(uint8_t(0x00)); // Configuration Register
  Wire.write(uint8_t(0x01)); // 00000001 (Start conversion)
  return Wire.endTransmission();
}

/**************************************************************
  #showADCPortState > Show ADC ports state
**************************************************************/
void showADCPortState(Print & OUT_PORT) {
  uint8_t bitShift = B00000001, tmpComp;
  if (EEPROM_readAnything(30, anaDisabledBits) == sizeof(anaDisabledBits)) {
    OUT_PORT.println(F("ADC ports state :"));
    for (uint8_t i = 0; i < 8; i++) {
      OUT_PORT.print(F("ADC port "));
      OUT_PORT.print(i + 1);
      tmpComp = bitShift & anaDisabledBits;
      if (tmpComp == bitShift) {
        OUT_PORT.println(F(" : DISABLED"));
      } else {
        OUT_PORT.println(F(" : ENABLED"));
      }
      bitShift = bitShift << 1;
    }
  } else {
    OUT_PORT.println(F("Error reading ADC ports state"));
  }
}

/**************************************************************
  #readAdcValue > Read one ADC value
**************************************************************/
int16_t readAdcValue(uint8_t registerID) {
  Wire.beginTransmission(0x1D); // Talk to ADC128D818 - A0 and A1 to GND
  Wire.write(registerID); // Busy Status Register (Bit 1 = Not Ready > Waiting for the power-up sequence to end)
  Wire.endTransmission();
  Wire.requestFrom(0x1D, 2); // Read Register 0x20
  if (Wire.available() == 2) {
    return (Wire.read() << 4 | Wire.read()); // We read a 12 bits value (2x 8 bits I2C reads)
  }
  return -1;
}

/**************************************************************
  #readAdcValues > Read all ADC values (8)
**************************************************************/
void readAdcValues(int16_t anaValues[]) {
  uint8_t bitShift = B00000001, tmpComp;
  uint8_t registerID = 0x20; // First register ID is 0x20
  for (uint8_t i = 0; i < 8; i++) {
    tmpComp = bitShift & anaDisabledBits;
    if (tmpComp == bitShift) {
      anaValues[i] = -1;
    } else {
      anaValues[i] = readAdcValue(registerID);
    }
    registerID++;
    bitShift = bitShift << 1;
  }
}
