#include <Wire.h>
#include "Adafruit_TCS34725.h"

#define DT 3
#define COLOR_THRESHOLD 2
#define OUT 13
#define IN 12
#define PULSE_DELAY 750
#define PULSE_DURATION 300

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_60X);
uint16_t delta;
uint16_t threshold;

void setup(void) {
  Serial.begin(9600);
  Serial.print("Starting up");

  pinMode(OUT, OUTPUT);
  pinMode(IN, INPUT);
  
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
  uint16_t r, g, b, c;
  
  tcs.getRawData(&r, &g, &b, &c);
  delta = r - b;
  threshold = r + 20;
}

unsigned long lastBallTime = 0;

void loop(void) {
  uint16_t r, g, b, c;
  
  //Serial.print("Reading 1"); tcs.getRawData(&r, &g, &b, &c); Serial.println("\tDone");
  tcs.getRawData(&r, &g, &b, &c);
//  Serial.println(millis());
  
  Serial.println(String(b) + "\t" + String(r) + "\t" + threshold);
  if (r > threshold) {
    uint16_t rTotal = 0;
    uint16_t bTotal = 0;
  
    do {
      rTotal += r;
      bTotal += (b + delta);
      //Serial.print("Reading 2"); tcs.getRawData(&r, &g, &b, &c); Serial.println("\tDone");
      tcs.getRawData(&r, &g, &b, &c);
    } while (r > threshold);

    boolean filterRed = digitalRead(IN);
    if ((filterRed && rTotal >= bTotal) || (!filterRed && bTotal > rTotal)) {
      lastBallTime = millis();
      digitalWrite(OUT, HIGH);
    }
  }

  unsigned long elapsed = millis() - lastBallTime;
  
  if (elapsed > PULSE_DURATION) {
    digitalWrite(OUT, LOW);
  }

  //Serial.print(String(b) + "\t" + String(r) + "\t" + String(g) + "\n");// return;
}

