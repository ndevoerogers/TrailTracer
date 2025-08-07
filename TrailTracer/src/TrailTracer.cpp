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
#include "Wire.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_GFX.h"
#include "neopixel.h"
#include "Colors.h"


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
///OLED
const int OLED = -1;
const int OLEDADDRESS = 0x3C;
//neopixel
int p;
const int PIXELCOUNT = 12;
unsigned int currentTimePixels, lastTimePixels;


//variables for MPU6050
const float ACCEL_SCALE_FACTOR = 16384.0; //defauly for +-2g range
const float GRAVITY = 9.80665; //standard gravity in m/s^2
const float PERIOD_MS = 10; //period in milliseconds
const float DURATION_S = 5; //duration in seconds
float totalShock;
float crashThreshold;
int16_t accel_x,accel_y,accel_z;
float gx,gy,gz;

String dateTime, timeOnly; //MPU6050
unsigned int mpuLastTime; //mpu6050

//declare functions 
void MQTT_connect(); //publishing
bool MQTT_ping(); //publishing

void getGPS(float *latittude, float *longitude, int *satellites, float *speed);
void createEventPayLoad (float lat, float lon);
void pixelFill(int start, int end, int color);

Adafruit_SSD1306 display(OLED);


SYSTEM_MODE(AUTOMATIC);
//Adafruit_NeoPixel pixel(PIXELCOUNT, SPI1, WS2812B); // Works with Photon 2
Adafruit_NeoPixel pixel(PIXELCOUNT, D2, WS2812B); // Works with Photon 2


void setup() {

Serial.begin(9600);
waitFor(Serial.isConnected,10000);
display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
Wire.begin(); //intialize mpu
Wire.beginTransmission(0x68);
Wire.write(0x6B); //power management register
Wire.write(0x00); //wake up device
Wire.endTransmission(true);
//set time zone
Time.zone(-6);
Particle.syncTime(); //sync time with particle cloud
  


// intialize wifi
//  WiFi.on();
//  WiFi.connect();
// while(WiFi.connecting()) {
//  Serial.printf(".");
//  Serial.printf("\n\n");
//}


// intialize GPS
GPS.begin (0x10);
GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
GPS.sendCommand(PGCMD_ANTENNA);
delay (1000);
GPS.println(PMTK_Q_RELEASE);

  //Set accelerometer range +-2g range
Wire.beginTransmission(0x68);
Wire.write(0x1C); //accelerometer configuration register
Wire.write(0x00); //+-2g range
Wire.endTransmission(true);

// // intialize OLED 
display.begin(SSD1306_SWITCHCAPVCC, OLEDADDRESS);
display.setTextSize(1);
display.setRotation(0);
display.setTextColor(WHITE);

pixel.begin(); //intialize neopixels

}


void loop() {

display.clearDisplay();
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
  display.setCursor (0,0);
  display.printf(" Lat: %0.6f\n Lon: %0.6f\n Alt: %0.6f\n Satellites: %i\n Speed: %0.2f\n",lat, lon, alt, sat, speed);
  display.display();
  if((millis()-lastTime > 6000)) {
    if(mqtt.Update()) {
    
    createEventPayLoad(lat, lon);

      //TrailGPS.publish(ttGPS);
      Serial.printf("Publishing %f\n", TrailGPS);
    }
  lastTime = millis();
  }

}


dateTime = Time.timeStr(); //current date and time from particle cloud
timeOnly = dateTime.substring(11,19); //extracting time from dateTime
if (millis()- mpuLastTime > 100000) {
  mpuLastTime = millis();
}


Time.now();
Wire.beginTransmission(0x68);
Wire.write (0x3B); //start readig from ACCEL_XOUT_H
Wire.endTransmission (false);
Wire.requestFrom(0x68, 6,true);
if (Wire.available() == 6) {

  accel_x = ((Wire.read() << 8 | Wire.read()));
  accel_y = ((Wire.read() << 8 | Wire.read()));
  accel_z = ((Wire.read() << 8 | Wire.read()));
  gx=accel_x/ACCEL_SCALE_FACTOR;
  gy=accel_y/ACCEL_SCALE_FACTOR;
  gz=accel_z/ACCEL_SCALE_FACTOR;

  totalShock = sqrt((gx * gx) + (gy * gy) + (gz * gz));
  //Serial.printf("Total Shock %f\n", totalShock);
  Serial.printf("Total Shock %f:%f:%f:%f\n", totalShock, gx,gy, gz);

  crashThreshold = 2; //in g's

  if (totalShock >= crashThreshold) {
   if (mqtt.Update()) {
    TrailCrash.publish("Possible Crash- Check GPS");
    delay(2000);
   }
    
  }

}
 
  if ((currentTimePixels - lastTimePixels)>10000) {

  pixel.setBrightness(30);
  pixelFill(0, 12,red);
  pixel.show();
  lastTimePixels = millis();
  }
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

void pixelFill(int start, int end, int color) {
  int p;
  for (p=start; p<=end; p++) {
    pixel.setPixelColor (p, color);
  }
}