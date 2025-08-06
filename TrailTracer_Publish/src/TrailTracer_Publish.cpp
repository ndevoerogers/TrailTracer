/* 
 * Project TrailTracer_Publish
 * Author: Nicole DeVoe Rogers
 * Date: 5 August 2025
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */


#include "Particle.h"
#include <Adafruit_GPS.h>
#include "GPS_CNM.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_SPARK.h"
#include "credentials.h"

// *********MQTT Setup
TCPClient TheClient; //transmission control
Adafruit_MQTT_SPARK mqtt(&TheClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

Adafruit_GPS GPS(&Wire); 
//**********creating a publish_object and linking to a feed */
Adafruit_MQTT_Publish TrailGPS = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME"/feeds/trailtracergps");
Adafruit_MQTT_Publish TrailSpeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/trailtracerspeed");
Adafruit_MQTT_Publish TrailCrash = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/trailtracercrash");

//declare variables
unsigned int last, lastTime;
const int TIMEZONE = -6;
const unsigned int UPDATE = 30000;

float lat, lon, alt, speed;
int sat;
unsigned int lastGPS;

//publishing
float ttGPS;
float ttSpeed;
float ttCrash; 

//declare functions
void MQTT_connect();
bool MQTT_ping();

void getGPS(float *latitude, float *longitude, float *altitude, int *satellites, float *speed);


SYSTEM_MODE(SEMI_AUTOMATIC); //connecting to internet but not particle cloud


void setup() {

  Serial.begin();
  waitFor(Serial.isConnected, 10000);

  //connecting to internet
  WiFi.on();
  WiFi.connect();
  while (WiFi.connecting()) {
    Serial.printf(".");
    Serial.printf("\n\n");
  }

  // intialize GPS
  GPS.begin (0x10);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay (1000);
  GPS.println(PMTK_Q_RELEASE);


}

void loop() {
  


  GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
      return;
    }
  }

  MQTT_connect();
  MQTT_ping();

  if (millis() - lastGPS > UPDATE) {
    lastGPS = millis(); //reset timer
    
    speed = speed * 1.151; //converts from knots to mph
    getGPS(&lat,&lon,&alt,&sat,&speed);

  if((millis()-lastTime > 6000)) {
    if(mqtt.Update()) {
    


      TrailGPS.publish(ttGPS);
      Serial.printf("Publishing %f\n", TrailGPS);
    }
  lastTime = millis();
  }
  
}
}

void getGPS(float *latitude, float *longitude, float *altitude, int *satellites, float *speed) {
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
void MQTT_connect() {
  int8_t ret;

  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}

bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}
