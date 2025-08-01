/* 
 * Project TrailTracer
 * Author: Nicole DeVoe Rogers
 * Date: 1 August 2025
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */


#include "Particle.h"
#include <Adafruit_GPS.h>
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_GFX.h"
#include "GPS_CNM.h"
#include "credentials.h"

TCPClient TheClient;

// Constants
const int TIMEZONE = -6;
const unsigned int UPDATE = 30000;

const int OLED = -1;
const int ADDRESS = 0x76;
const int OLEDADDRESS = 0x3C;

// variables
float lat, lon, alt, speed;
int sat;
unsigned int lastGPS;

Adafruit_GPS GPS(&Wire);
Adafruit_MQTT_SPARK mqtt(&TheClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_SSD1306 display (OLED);

void getGPS(float *latitude, float *longitude, float *altitude, int *satellites, float *speed);
SYSTEM_MODE(SEMI_AUTOMATIC);


void setup() {
 
  Serial.begin(9600);
  waitFor(Serial.isConnected,5000);
  
  // intialize GPS
  GPS.begin (0x10);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay (1000);
  GPS.println(PMTK_Q_RELEASE);

  // intialize OLED 
  display.begin(SSD1306_SWITCHCAPVCC, OLEDADDRESS);
  display.setTextSize(1);
  display.setRotation(0);
  display.setTextColor(WHITE);

  delay(1000);
}


void loop() {
  display.clearDisplay();

  //data from GPS unit (do this continuously)
  GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
      return;
    }
  }

  if (millis() - lastGPS > UPDATE) {
    lastGPS = millis(); //reset timer
    getGPS(&lat,&lon,&alt,&sat,&speed);
    speed = speed * 1.151; //converts from knots to mph
    Serial.printf("Lat: %0.6f, Lon: %0.6f, Alt: %0.6f, Satellites: %i, Speed: %0.2f\n",lat, lon, alt, sat, speed);
    display.setCursor (0,0);
    display.printf(" Lat: %0.6f\n Lon: %0.6f\n Alt: %0.6f\n Satellites: %i\n Speed: %0.2f\n",lat, lon, alt, sat, speed);
    display.display();
  }
}

void getGPS(float *latitude, float *longitude, float *altitude, int *satellites, float *speed){
  int theHour;

  theHour = GPS.hour + TIMEZONE;
  if(theHour < 0) {
    theHour = theHour + 24;
  }
    
  Serial.printf("Time: %02i:%02i:%02i:%03i\n",theHour, GPS.minute, GPS.seconds, GPS.milliseconds);
  Serial.printf("Dates: %02i-%02i-20%02i\n", GPS.month, GPS.day, GPS.year);
  Serial.printf("Fix: %i, Quality: %i",(int)GPS.fix,(int)GPS.fixquality);
    if (GPS.fix) {
      *latitude = GPS.latitudeDegrees;
      *longitude = GPS.longitudeDegrees; 
      *altitude = GPS.altitude;
      *satellites = (int)GPS.satellites;
      *speed = GPS.speed;

    }
}
