/* 
 * Project: TrailTracer_Accel
 * Author: Nicole DeVoe Rogers
 * Date: 4 August 2025
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */


#include "Particle.h"
#include "Wire.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_GFX.h"

const float ACCEL_SCALE_FACTOR = 16384.0; // Default for ±2g range
const float GRAVITY = 9.80665;            // Standard gravity in m/s^2
const int OLED_RESET = -1;
const int SAMPLE_PERIOD_MS = 10;               // Sample period in milliseconds
const int SAMPLE_DURATION_S = 5;               // Sample duration in seconds
const int ARRAY_SIZE = SAMPLE_DURATION_S * 50; // Number of samples in array

String dateTime, timeOnly;
unsigned int lastTime;

Adafruit_SSD1306 display(OLED_RESET);
float accelArray[ARRAY_SIZE];
int arrayIndex = 0;

SYSTEM_MODE(AUTOMATIC); //automatic connects to particle cloud


void setup() {

  Serial.begin(9600);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  Wire.begin();
  // Initialize MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // Power management register
  Wire.write(0x00); // Wake up device
  Wire.endTransmission(true);
  // set time zone
  Time.zone(-6);
  Particle.syncTime(); //sync time with particle cloud
  // Set accelerometer range to ±2g
  Wire.beginTransmission(0x68);
  Wire.write(0x1C); // Accelerometer configuration register
  Wire.write(0x00); // ±2g range
  Wire.endTransmission(true);

}


void loop() {

  dateTime = Time.timeStr(); //current date and time from particle cloud
  timeOnly = dateTime.substring(11,19); //extract the time from dateTime
  if (millis() - lastTime > 100000) {
    lastTime = millis();
  }
  
  Time.now();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Start reading from ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);
  if (Wire.available() == 6){
  
    float accel_x = ((Wire.read() << 8 | Wire.read()) / ACCEL_SCALE_FACTOR) * GRAVITY;
    float accel_y = ((Wire.read() << 8 | Wire.read()) / ACCEL_SCALE_FACTOR) * GRAVITY;
    float accel_z = ((Wire.read() << 8 | Wire.read()) / ACCEL_SCALE_FACTOR) * GRAVITY;
    // Store acceleration sum in array
    accelArray[arrayIndex] = sqrt((accel_x * accel_x) + (accel_y * accel_y) + (accel_z * accel_z));
    // Update array index
    arrayIndex++;
    if (arrayIndex <= ARRAY_SIZE)
    {
      arrayIndex = 0; // Reset index for next cycle
    }
    // Find maximum value in array
    float maxAccel = accelArray[0];
    for (int i = 1; i < ARRAY_SIZE; i++)
    {
      if (maxAccel < accelArray[i])
      {
        maxAccel = accelArray[i];
      }
    }
    // Print maximum acceleration
    Serial.printf("Max acceleration: %.2f g\n", maxAccel / GRAVITY);
    // Display maximum acceleration on OLED screen
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    //display.print(Time.format("%A , %B %d"));
    display.printf("Date and Time is %s \n", dateTime.c_str());
    // display.setCursor(0, 20);
    // display.print(Time.format("%I : %M : %S %p"));
    display.setCursor(0, 40);
    display.printf("Max Accel: %.2f g", maxAccel / GRAVITY);
    display.display();
  }
  else
  {
    Serial.println("Error reading from MPU6050");
  }
  delay(SAMPLE_PERIOD_MS);
}
