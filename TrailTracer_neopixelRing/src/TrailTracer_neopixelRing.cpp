/* 
 * Project TrailTracer_neopixelRing
 * Author: Nicole DeVoe Rogers
 * Date: 4 August 2025
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

#include "Particle.h"
#include "neopixel.h"
#include "Colors.h"



SYSTEM_MODE(SEMI_AUTOMATIC);

int p;
const int PIXELCOUNT = 12;


void pixelFill(int start, int end, int color);

Adafruit_NeoPixel pixel (PIXELCOUNT, SPI1, WS2812B);

void setup() {

pixel.begin();

}

void loop() {

    pixel.setBrightness(30);
    pixelFill(0,12,red);
    pixel.show();
    delay (500);
    pixel.clear();
  
}
void pixelFill(int start, int end, int color){
  int p; 
  for (p=start; p<=end; p++){
    pixel.setPixelColor (p, color);
    }
  }