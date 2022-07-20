#include <FastLED.h>

#define NUM_LEDS 5
#define DATA_PIN 21

CRGB leds[NUM_LEDS];

void setup() {
  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);  // GRB ordering is typical
}

void loop() {
  // Turn the LED on, then pause
  for (int i = 0; i < NUM_LEDS; i++)
    leds[i] = CRGB(255, 0, 0);//緑
  FastLED.show();
  delay(500);
  for (int i = 0; i < NUM_LEDS; i++)
    leds[i] = CRGB(255, 255, 0);//黄
  FastLED.show();
  delay(500);
  for (int i = 0; i < NUM_LEDS; i++)
    leds[i] = CRGB(0, 255, 0);//赤
  FastLED.show();
  delay(500);
  // Now turn the LED off, then pause
  for (int i = 0; i < NUM_LEDS; i++)
    leds[i] = CRGB::Black;
  FastLED.show();
  delay(500);
}
