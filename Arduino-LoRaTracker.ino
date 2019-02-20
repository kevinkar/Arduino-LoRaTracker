#include <Adafruit_GPS.h>
#include <ArduinoLowPower.h>
#include <MKRWAN.h>
#include <RTCZero.h>
#include <stdlib.h>

/* WGS84 and GRS80 Ellipsoid reference */
#define EARTHRADIUS 6378137
/* Decimal accuracy for degree string. 5 decimals means 1.1m accuracy */
#define DEGREEDECIMALS 5
/* Fixed length of the degree string */
#define DEGREESTRLENGTH 10
/* Hardware serial port for GPS */
#define GPSSerial Serial1

/* Hardware pins */
#define PINGPSENABLE 0
#define PINGPSFIX 1
#define PINGPSPPS 2
#define LED 6
#define PINTAMPERING 4

/* Default timer intervals */
#define SYSTEMINTERVAL 10000
#define NETWORKINTERVAL 600000
#define GPSINTERVAL 10000

/* Wait a while to attempt to comply with duty cycle regulations */
#define LORA_JOIN_FAILED_RETRY_INTERVAL 60000
/* Modem firmware allows sending every 2 minutes. For duty cycle regulations: 4 minutes */
/* Depends on SF and payload size. 20 bytes SF12 => 1.8s, 20 bytes SF7 => 0.2s */
/* Max 2 seconds air time / 240 seconds => 0.83% */
#define LORA_JOIN_RETRIES_BEFORE_RESET 5
#define LORA_SEND_INTERVAL 240000
/* Between 1 and 255, 0 reserved for system/mac messages */
#define LORA_SEND_PORT 1

/* Status code bits */
#define BATTERY_LOW B00000001
#define TAMPERED B00000010
#define ACC_TRESHOLD B00001000
#define GEOFENCE_ALARM B00000100
#define GPS_FIX B00010000
#define ACTIVE_MODE B00100000
#define CHAR_OFFSET B01000000

/* LoRa connection values. Replace with your own */
#define APPEUI "0000000000000000"
#define APPKEY "00000000000000000000000000000000"

/* Geofence default position, Tuomiokirkko */
#define DEFAULTLATITUDE 60.45246;
#define DEFAULTLONGITUDE 22.27832;

/* Connect GPS over serial */
Adafruit_GPS GPS(&GPSSerial);

/* LoRa variables */
LoRaModem modem;
String appEui;
String appKey;
String devAddr;
int messageSendStatus;
bool loraJoined;
int loraJoinRetries;
String messageToSend;

/* Geofence variables for radial region */
uint16_t geofenceRadius; // Meters
float homeLatitude; // DD.ddddd
float homeLongitude; //DD.ddddd
float distanceHome; // Meters

/* Last known coordinates globally and our status */
float lastLatitude; // DD.ddddd
float lastLongitude; // DD.ddddd
byte status;

/* RTC functionality */
RTCZero rtc;
int alarm;

/* Timers for different parts */
uint32_t systimer;
uint32_t nettimer;
uint32_t gpstimer;

uint32_t systemInterval;
uint32_t networkInterval;
uint32_t gpsInterval;

/* Board Reset */
void(* resetFunction) (void) = 0;

/*
    Setup everything on startup
*/
void setup() {
  delay(5000);
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  initGPS();

  geofenceRadius = 1000;
  homeLatitude = DEFAULTLATITUDE;
  homeLongitude = DEFAULTLONGITUDE;
  distanceHome = 0.00;

  lastLatitude = 0.00;
  lastLongitude = 0.00;
  status = B00000000;

  appEui = APPEUI;
  appKey = APPKEY;

  initLoRa();
  delay(1000);

  initRTC();
  /* Millisecond timers */
  systimer = millis();
  nettimer = millis();
  gpstimer = millis();
  systemInterval = SYSTEMINTERVAL;
  networkInterval = NETWORKINTERVAL;
  gpsInterval = GPSINTERVAL;
  /* Sends a first message on startup */
  //sendMessage(formatMessage(lastLatitude, lastLongitude));
  alarm = 1;
  digitalWrite(LED, LOW);
}

/*
   Main functionality here.
*/
void loop() {
  /* Go to sleep */
  if (alarm == 0) {
    modem.sleep(); // Retains network join => very necessary, otherwise send will always fail after sleep
    GPS.standby();
    GPSSerial.end();
    digitalWrite(PINGPSENABLE, LOW);
    rtcSetAlarmIn(0, 55, 0); // Own overlay method
    /* While https://github.com/arduino-libraries/RTCZero/pull/42 is not yet accepted */
    /* Use the LowPower libraries sleep due to its improved capabilities */
    rtc.standbyMode();
    //LowPower.sleep();
    // Sleep until interrupt
    digitalWrite(LED, HIGH);
    Serial.println("Just woke up");
    modem.sleep(false);
    initGPS();
    //digitalWrite(PINGPSENABLE, HIGH);
    //delay(600);
    //GPS.begin(9600);
    //GPS.wakeup();
    systimer = millis();
    digitalWrite(LED, LOW);
  }
  /* Needs to "continuosly" read the GPS to receive sentence data */
  while (!GPS.newNMEAreceived()) {
    /* The GPS library handles the characters, no need to bring them through */
    GPS.read();
  }
  /* Note that this clear the receivedflag */
  Serial.println(GPS.lastNMEA());
  /* Parses last received sentence (dependant on read to receive) */
  if (!GPS.parse(GPS.lastNMEA())) {
    return;
  }
  /*  Turn on LED while working */
  digitalWrite(LED, HIGH);

  /* Resets timers if millis() or timer wraps around */
  if (systimer > millis()) systimer = millis();
  if (nettimer > millis()) nettimer = millis();
  if (gpstimer > millis()) gpstimer = millis();

  printGPSDataTime();
  if (GPS.fix) {
    // TODO Could probably set real RTC time here
    printGPSDataLocation();
    addState(GPS_FIX);
    lastLatitude = GPS.latitudeDegrees;
    lastLongitude = GPS.longitudeDegrees;
    distanceHome = distance(lastLatitude, lastLongitude, homeLatitude, homeLongitude);
    Serial.print(distanceHome, 4); Serial.print(" meters home.");
    if (distanceHome > geofenceRadius && (lastLatitude != 0 || lastLongitude != 0)) {
      addState(GEOFENCE_ALARM);
    } else {
      removeState(GEOFENCE_ALARM);
    }
    sendAndPrepareSleep();
    /* Jump out and start loop from beginning */
    return;
  } else {
    removeState(GPS_FIX);
  }
  /* Have been awake for 5 minutes */
  if (millis() - systimer > 300000) {
    Serial.println("No fix, timed out.");
    sendAndPrepareSleep();
    /* Jump out and start loop from beginning */
    return;
  }
  digitalWrite(LED, LOW);
}

void sendAndPrepareSleep() {
  /* Send coordinates and then go to sleep */
  messageToSend = formatMessage(lastLatitude, lastLongitude);
  Serial.print("Sending message: "); Serial.println(messageToSend);
  sendMessage(messageToSend);
  digitalWrite(LED, LOW);
  alarm = 0;
}

/*
   Initializes the GPS module for use
*/
void initGPS() {
  pinMode(PINGPSENABLE, OUTPUT);
  pinMode(PINGPSFIX, INPUT);
  pinMode(PINGPSPPS, INPUT);
  // Needs at least 600ms delay between setting pin and starting GPS or it will not take the commands.
  digitalWrite(PINGPSENABLE, HIGH);
  delay(600);
  GPS.begin(9600);
  /* Minimum GPS data */
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  /* SET_FIX command must be sent before NMEA_UPDATE or else NMEA_UPDATE won't have any effect  */
  /* Fix MUST update more often than 100 mHz for the GPS to work properly */
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
  /* 10 second update rate for output */
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ);
}

/*
   Initializes the LoRa modem and connects to the network
*/
bool initLoRa() {
  appKey.trim();
  appEui.trim();
  loraJoined = false;
  loraJoinRetries = 0;
  messageSendStatus = 0;
  Serial.println("Starting LoRa modem");
  if (!modem.begin(EU868)) {
    Serial.println("Failed to start LoRa modem. Resetting board...");
    resetFunction();
  }
  Serial.print("LoRa modem version: "); Serial.println(modem.version());
  Serial.print("DevEUI: "); Serial.println(modem.deviceEUI());
  Serial.print("OTAA Connect AppEUI:"); Serial.print(appEui);
  Serial.print(" AppKey:"); Serial.println(appKey);
  Serial.println("Connecting");
  loraJoined = modem.joinOTAA(appEui, appKey);
  loraJoinRetries++;
  while (!loraJoined && (loraJoinRetries++ <= LORA_JOIN_RETRIES_BEFORE_RESET) ) {
    Serial.println("Failed to connect. Retrying in a moment.");
    delay(LORA_JOIN_FAILED_RETRY_INTERVAL);
    Serial.println("Retrying...");
    loraJoined = modem.joinOTAA(appEui, appKey);
  }
  if (loraJoinRetries > LORA_JOIN_RETRIES_BEFORE_RESET ) {
    resetFunction();
  }
  modem.setPort(LORA_SEND_PORT);
  Serial.print("Connected to network!");
  return true;
}

/*
   Send message over LoRa
*/
void sendMessage(String message) {
  if (loraJoined) {
    messageSendStatus = 0;
    modem.beginPacket();
    modem.print(message);
    messageSendStatus = modem.endPacket(true);
    if (messageSendStatus > 0) {
      Serial.println("Message \"" + message + "\" sent successfully!");
    } else {
      Serial.println("Error sending message \"" + message);
    }
  } else {
    Serial.println("LoRa connection not available");
  }
}

/*
   Initializes the RTC for use with relative time
*/
void initRTC() {
  alarm = 0;
  rtc.begin();
  rtc.setTime(0, 0, 0);
  rtc.setDate(1, 1, 19);
  /* Match on clock value regardless of date */
  rtc.enableAlarm(rtc.MATCH_HHMMSS);
  rtc.attachInterrupt(ISR_RTCAlarm);
}

/*
   ISR called on RTC alarm
*/
void ISR_RTCAlarm() {
  alarm++;
}

/*
   Verify a date. Does not consider the actual number of valid days in a month.
   Returns true for dates from 1.1.2000 to 31.12.2050
   Year can be given with or without millenia prefix.
*/
bool validDate(int day, int month, int year) {
  if (day < 1 || day > 31) {
    return false;
  }
  if (month < 1 || month > 12) {
    return false;
  }
  if ( year < 0 || year > 2050 || (year < 2000 && year > 50)) {
    return false;
  }
}

/*
   Verify the hours, minutes and seconds are on the clock.
   Returns true if time is between 00:00:00 and 23:59:59
*/
bool validTime(int hour, int minute, int second) {
  if (hour < 0 || hour > 23 ) {
    return false;
  }
  if (minute < 0 || minute > 59) {
    return false;
  }
  if (second < 0 || second > 59) {
    return false;
  }
  return true;
}

/*
   Helper for the RTC alarm clock. Checks the current time and adds the time with respect to valid clock times.
   RTC needs to be configured for daily alarms since this only considers time and will spin around.
   Will not add more than 23:59:59.
   Returns true if succesful
*/
bool rtcSetAlarmIn(int hours, int minutes, int seconds) {
  if (!validTime(hours, minutes, seconds)) {
    return false;
  }
  int currentHours = rtc.getHours();
  int currentMinutes = rtc.getMinutes();
  int currentSeconds = rtc.getSeconds();
  /* RTC in a bad state. Should consider reset? */
  if (!validTime(currentHours, currentMinutes, currentSeconds)) {
    return false;
  }
  /* Forward overflow addition */
  seconds += currentSeconds;
  if (seconds > 59) {
    seconds -= 60;
    minutes++;
  }
  minutes += currentMinutes;
  if (minutes > 59) {
    minutes -= 60;
    hours++;
  }
  hours += currentHours;
  if (hours > 23) {
    hours -= 24;
  }
  /* This should not be necessary */
  if (!validTime(hours, minutes, seconds)) {
    return false;
  }
  rtc.setAlarmTime(hours, minutes, seconds);
  return true;
}

/*
   Double to String from AVR library
   Width - minimun length of output string including negative sign and decimal dot. Pads with spaces.
   Precision is zero-padded after decimal dot
*/
char *dtostrf(double val, signed char width, unsigned char prec, char *sout) {
  char fmt[20];
  sprintf(fmt, "%%%d.%df", width, prec);
  sprintf(sout, fmt, val);
  return sout;
}

/*
   Float to String
*/
String floatToString(float value) {
  char buff[DEGREESTRLENGTH + 1];
  String valueString = "";
  dtostrf(value, DEGREESTRLENGTH, DEGREEDECIMALS, buff);
  valueString += buff;
  return valueString;
}

/*
   Returns string with coordinates as defined in the message protocol.
*/
String formatMessageCoordinates(float latitude, float longitude) {
  char coord[6];
  int32_t lat = 0;
  int32_t lon = 0;

  latitude += 180;
  longitude += 180;
  latitude = latitude * 10000.0;
  longitude = longitude * 10000.0;
  lat = (int) round(latitude);
  lon = (int) round(longitude);

  /* (Extra) shifts to clear any unwanted values. */
  coord[0] = (lat << 8) >> 24;
  coord[1] = (lat << 16) >> 24;
  coord[2] = (lat << 24) >> 24;

  coord[3] = (lon << 8) >> 24;
  coord[4] = (lon << 16) >> 24;
  coord[5] = (lon << 24) >> 24;

  String coordinates = "";

  for(int i=0;i<6;i++) {
    coordinates += coord[i];
  }
  return coordinates;
}

/*
   Returns string with message containing status and coordinates as defined in the message protocol.
*/
String formatMessage(float latitude, float longitude) {
  String message = "";
  message += statusToChar();
  message += formatMessageCoordinates(latitude, longitude);
  return message;
}

void addState(byte state) {
  status |= state;
}

void removeState(byte state) {
  status &= ~state;
}

boolean checkState(byte state) {
  return (status & state);
}

/*
   Status byte converted to an ASCII char.
*/
char statusToChar() {
  return (char) status | CHAR_OFFSET;
}

/*
   Great circle calculation for the distance between two points
   Converts to radians, calculates and outputs distance in meters
*/
float distance(float lat1, float lon1, float lat2, float lon2) {
  lat1 = radians(lat1);
  lon1 = radians(lon1);
  lat2 = radians(lat2);
  lon2 = radians(lon2);
  float temp = acos(sin(lat1) * sin(lat2) + cos(lat1) * cos(lat2) * cos(lon1 - lon2)) * EARTHRADIUS;
  return temp;
  /*
     Alternative, more accurate? formula
     float temp = 2*asin(sqrt(sq(sin((lat1 - lat2) / 2)) + cos(lat1) * cos(lat2) * sq((sin((lon1 - lon2) / 2)))))*EARTHRADIUS;
  */
}

/* Long printing moved here */
void printGPSDataTime() {
  Serial.print("\nTime: ");
  Serial.print(GPS.hour, DEC); Serial.print(':');
  Serial.print(GPS.minute, DEC); Serial.print(':');
  Serial.print(GPS.seconds, DEC); Serial.print('.');
  Serial.println(GPS.milliseconds);
  Serial.print("Date: ");
  Serial.print(GPS.day, DEC); Serial.print('/');
  Serial.print(GPS.month, DEC); Serial.print("/20");
  Serial.println(GPS.year, DEC);
  Serial.print("Fix: ");
  Serial.print((int)GPS.fix);
  Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
}

/* Long printing moved here */
void printGPSDataLocation() {
  Serial.print("Location: ");
  Serial.print(GPS.latitudeDegrees, 6); Serial.print(GPS.lat);
  Serial.print(", ");
  Serial.print(GPS.longitudeDegrees, 6); Serial.println(GPS.lon);
  //Serial.print("1/100000 of degrees");
  //Serial.print(GPS.latitude_fixed); Serial.print(","); Serial.println(GPS.longitude_fixed);
  Serial.print("Speed (knots): "); Serial.println(GPS.speed);
  Serial.print("Angle: "); Serial.println(GPS.angle);
  Serial.print("Altitude: "); Serial.println(GPS.altitude);
  Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
}
