
/// Interrupt-triggered "Motion Activated" NeoPixel Light for ATTiny85
// Created by: David R Ratliff
// October, 2013
// In the public domain
// Created, written, and tested on Digispark ATTiny85 based Arduino compatible platform
// This sketch simply sets an interrupt on pin, puts ATTiny to sleep...
// waits for interrrupt, and runs 'light show'
// Interupt was designed for an active HIGH PIR sensor, but can use anything which outputs
// a digital high signal to trigger the interrupt and activate the NeoPixel program
// 


#include <Adafruit_NeoPixel.h>      // Library for Adafruit NeoPixels
#include <avr/sleep.h>              // Library for ATTiny 'sleep' functionality

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define PIN 0          // Pin on which NeoPixels are attached

byte PIR = 5;          // Interrupt pin
byte LED = 1;          // Status/debugging LED pin (turns on when sleep mode entered)
                       // This should probably be disabled for energy efficiency!

// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_RGB     Pixels are wired for RGB bitstream
//   NEO_GRB     Pixels are wired for GRB bitstream
//   NEO_KHZ400  400 KHz bitstream (e.g. FLORA pixels)
//   NEO_KHZ800  800 KHz bitstream (e.g. High Density LED strip)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(8, PIN, NEO_GRB + NEO_KHZ800);

void setup() 
{
  pinMode(PIR, INPUT);
  digitalWrite(PIR, LOW);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  sbi(GIMSK, PCIE);      // Turn on Pin Change Interrupt on port
  sbi(PCMSK, PCINT5);    // Which particular pin affected by interrupt


  strip.begin();
  strip.show();          // Initialize all pixels to 'off'
  delay(1000);           // Small pause on startup
  colorWipe(55, 10);     // Simple color display to signal that everything is working! YAY!
  strip.show(); 

  delay(20000);          // give PIR time to "settle" before initiating sleep & interrupt
}

void loop() 
{
  // What do you want to happen while 'awake'
  if(digitalRead(PIR))    // IF motion activity - (PIR pin HIGH)
  {
    rainbowCycle(10);     // show rainbow cycle on NeoPixels  (All colors at same time)
    rainbowCycle(5);      // show rainbow cycle on NeoPixels
    rainbowCycle(0);      // show rainbow cycle on NeoPixels
    return;
  }

  else                     // If no motion - after motion stops
  {
    rainbow(15);           // signals about to shut off  (one color at a time)
    colorWipe((0),0);      // turn them off
    system_sleep();        // do nothing...prepare for sleep
  }

}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } 
  else if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } 
  else {
    WheelPos -= 170;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

void system_sleep()                // Goin' to bed!
{
  digitalWrite(LED, HIGH);        // Turn onboard LED ON to indicate sleep mode (Debug feedback)
  delay(20);
  cbi(ADCSRA,ADEN);                        // Switch analog to digital converter OFF
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);     // set sleep mode
  sleep_mode();                            // system sleeps here
  sbi(ADCSRA,ADEN);                        // switch ADC ON - after interrupt wakes up ATTiny
  digitalWrite(LED, LOW);                  // Turns onbaord LED OFF at wake-up (more debug feedback)


}

ISR(PCINT0_vect)
{

}





