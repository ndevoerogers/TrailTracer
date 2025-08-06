/* 
 * Project TrailTracer
 * Author: Nicole DeVoe Rogers
 * Date: 1 August 2025
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */


#include "Particle.h"
#include <Adafruit_GPS.h>
#include "GPS_CNM.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_SPARK.h"
#include "JsonParserGeneratorRK.h"
#include "credentials.h"

TCPClient TheClient;
Adafruit_MQTT_SPARK mqtt(&TheClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_GPS GPS(&Wire);


Adafruit_MQTT_Publish TrailGPS = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME"/feeds/trailtracergps");
Adafruit_MQTT_Publish TrailSpeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/trailtracerspeed");
Adafruit_MQTT_Publish TrailCrash = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/trailtracercrash");

unsigned int last, lastTime;
const int TIMEZONE = -6;
const unsigned int UPDATE = 30000;


// variables for adafruit mini gps
float lat, lon, alt, speed;
int sat;
unsigned int lastGPS;

//publishiing
float ttGPS;
float ttSpeed;
float ttCrash;

//declare functions 
void MQTT_connect(); //publishing
bool MQTT_ping(); //publishing

void getGPS(float *latittude, float *longitude, int *satellites, float *speed);
void createEventPayLoad (float lat, float lon);






SYSTEM_MODE(AUTOMATIC);


void setup() {

  Serial.begin(9600);
  waitFor(Serial.isConnected,10000);



// intialize wifi
   WiFi.on();
  WiFi.connect();
  while(WiFi.connecting()) {
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

// // intialize OLED 
// display.begin(SSD1306_SWITCHCAPVCC, OLEDADDRESS);
// display.setTextSize(1);
// display.setRotation(0);
// display.setTextColor(WHITE);

}


void loop() {

  // display.clearDisplay();
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
    getGPS(&lat,&lon,&sat,&speed);
    Serial.printf("Lat: %0.6f, Lon: %0.6f, Satellites: %i, Speed: %0.2f\n",lat, lon, sat, speed);
    // display.setCursor (0,0);
    // display.printf(" Lat: %0.6f\n Lon: %0.6f\n Alt: %0.6f\n Satellites: %i\n Speed: %0.2f\n",lat, lon, alt, sat, speed);
    // display.display();
    if((millis()-lastTime > 6000)) {
    if(mqtt.Update()) {
    
    createEventPayLoad(lat, lon);

      TrailGPS.publish(ttGPS);
      Serial.printf("Publishing %f\n", TrailGPS);
    }
  lastTime = millis();
  }

}
//   currentTime = millis ();
// if ((currentTime - lastTime)>10000) {
//   lastTime = millis();
}



void getGPS(float *latitude, float *longitude, int *satellites, float *speed) {
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
  // *altitude = GPS.altitude;
  *satellites = (int)GPS.satellites;
  *speed = GPS.speed;

  }

}

void createEventPayLoad (float lat, float lon) {
  JsonWriterStatic <256> jw;
  {

    JsonWriterAutoObject obj (&jw);

    jw.insertKeyValue("lat" , lat);
    jw.insertKeyValue("lon", lon);
  }
 TrailGPS.publish(jw.getBuffer());
}


void MQTT_connect() {
   int8_t ret;
 
 // Return if already connected.
  if (mqtt.connected()) {
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