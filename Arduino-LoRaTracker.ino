#include <Adafruit_GPS.h>
#include <MKRWAN.h>
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

/* Status code bits */
#define BATTERY_LOW B00100000
#define TAMPERED B00010000
#define ACC_TRESHOLD B00001000
#define GEOFENCE_ALARM B00000100
#define GPS_FIX B00000010
#define ACTIVE_MODE B00000001
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
int messageSuccess;
int loraConnected;

/* Geofence variables for radial region */
uint16_t geofenceRadius; // Meters
float homeLatitude; // DD.ddddd
float homeLongitude; //DD.ddddd
float distanceHome; // Meters

/* Last known coordinates globally and our status */
float lastLatitude; // DD.ddddd
float lastLongitude; // DD.ddddd
byte status;

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
void setup()
{
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  initGPS();

  geofenceRadius = 500;
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
  /* Millisecond timers */
  systimer = millis();
  nettimer = millis();
  gpstimer = millis();
  systemInterval = SYSTEMINTERVAL;
  networkInterval = NETWORKINTERVAL;
  gpsInterval = GPSINTERVAL;
  /* Sends a first message on startup */
  sendMessage(formatMessage(lastLatitude, lastLongitude));
  digitalWrite(LED, LOW);
}

/*
   Main functionality here.
*/
void loop()
{
  /* Needs to "continuosly" read the GPS to receive sentence data */
  while (!GPS.newNMEAreceived()) {
    /* The GPS library handles the characters, no need to bring them through */
    GPS.read();
    //char c = GPS.read();
    //Serial.print(c);
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

  /* This is not needed if GPS is set for 10 second update interval */
  //if (millis() - gpstimer > gpsInterval) {
  gpstimer = millis();
  printGPSDataTime();
  if (GPS.fix) {
    printGPSDataLocation();
    addState(GPS_FIX);
    lastLatitude = GPS.latitudeDegrees;
    lastLongitude = GPS.longitudeDegrees;
    distanceHome = distance(lastLatitude, lastLongitude, homeLatitude, homeLongitude);
    if (distanceHome > geofenceRadius && (lastLatitude != 0 || lastLongitude != 0)) {
      addState(GEOFENCE_ALARM);
    } else {
      removeState(GEOFENCE_ALARM);
    }
    Serial.print(distanceHome, 4); Serial.print(" meters home.");
  } else {
    removeState(GPS_FIX);
  }

  /* LoRa FW only allows to send every 2 minutes */
  if (millis() - nettimer > networkInterval) {
    nettimer = millis();
    String messageToSend = formatMessage(lastLatitude, lastLongitude);
    Serial.print("Sending message: "); Serial.println(messageToSend);
    sendMessage(messageToSend);
  }
  //}
  digitalWrite(LED, LOW);
  /* Can be activated to avoid some while-looping above */
  delay(GPSINTERVAL);
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
  /* 10 second update rate for updates and fix */
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ);
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_200_MILLIHERTZ);
}

/*
   Initializes the LoRa modem and connects to the network
*/
void initLoRa() {
  appKey.trim();
  appEui.trim();
  loraConnected = 0;
  messageSuccess = 0;
  Serial.println("Starting LoRa modem");
  if (!modem.begin(EU868)) {
    Serial.println("Failed to start module. Resetting board...");
    resetFunction();
  }
  Serial.print("LoRa version: ");
  Serial.println(modem.version());
  Serial.print("DevEUI: ");
  Serial.println(modem.deviceEUI());
  Serial.print("OTAA Connect AppEUI:");
  Serial.print(appEui);
  Serial.print(" AppKey:");
  Serial.println(appKey);
  while (!loraConnected) {
    Serial.println("Connecting");
    loraConnected = modem.joinOTAA(appEui, appKey);
    Serial.println("Failed to connect. Retrying in 10 seconds.");
    delay(10000);
    Serial.println("Retrying...");
  }
  Serial.print("Connected to network!");
}

/*
   Send message over LoRa
*/
void sendMessage(String message) {
  if (loraConnected) {
    messageSuccess = 0;
    modem.setPort(3);
    modem.beginPacket();
    modem.print(message);
    messageSuccess = modem.endPacket(true);
    if (messageSuccess > 0) {
      Serial.println("Message \"" + message + "\" sent successfully!");
    } else {
      Serial.println("Error sending message \"" + message);
    }
  } else {
    Serial.println("LoRa connection not available");
  }
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
   Returns string with coordinates
*/
String formatMessageCoordinates(float latitude, float longitude) {
  return floatToString(latitude) + "," + floatToString(longitude);
}

/*
   Returns string with message containing status and coordinates in suitable format.
*/
String formatMessage(float latitude, float longitude) {
  return (String)statusToChar() + ";" + formatMessageCoordinates(latitude, longitude);
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
float distance(float lat1, float lon1, float lat2, float lon2)
{
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
