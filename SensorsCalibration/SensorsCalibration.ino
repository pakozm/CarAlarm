/*
 * This file is part of CarAlarm an Arduino project for a car alarm system.
 *
 * Copyright (c) 2016 Francisco Zamora-Martinez (pakozm@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#include <JeeLib.h>
ISR(WDT_vect) { Sleepy::watchdogEvent(); } // Setup for low power waiting
#include <SensorTransformations.h>

const unsigned long PERIOD_SLEEP = 100; // 100 ms

// digital pins connection
const int LEDS_ARRAY_LEN = 4;
const int LEDS_ARRAY_PIN[] = {7, 8, 9, 10};
const int BUZ_PIN = 11;
const int LED_PIN = 12;
const int PIR_PIN = 2;
const int TLT_PIN = 3;

// analogic pins connection
const int ACC_X_PIN = 2;
const int ACC_Y_PIN = 1;
const int ACC_Z_PIN = 0;
const int TEMP_PIN  = 5;

long Vcc; // in mili-volts

/////////////////////////////////////////////////////////////////////////////

void led_on() {
  digitalWrite(LED_PIN, HIGH);
}

void led_off() {
  digitalWrite(LED_PIN, LOW);
}

void buzzer_on() {
  digitalWrite(BUZ_PIN, HIGH);
}

void buzzer_off() {
  digitalWrite(BUZ_PIN, LOW);
}

/////////////////////////////////////////////////////////////////////////////

void blink(unsigned long ms=100, unsigned long post_ms=200) {
  led_on(); delay(ms);
  led_off(); delay(post_ms);
}

void buzz(unsigned long ms=100, unsigned long post_ms=200) {
  buzzer_on(); delay(ms);
  buzzer_off(); delay(post_ms);
}

void leds_array_blink() {
  for (int i=0; i<LEDS_ARRAY_LEN; ++i) digitalWrite(LEDS_ARRAY_PIN[i], HIGH);
  delay(50);
  for (int i=0; i<LEDS_ARRAY_LEN; ++i) digitalWrite(LEDS_ARRAY_PIN[i], LOW);
}

///////////////////////////////////////////////////////////////////////////////

void setup()
{
  for (int i=0; i<LEDS_ARRAY_LEN; ++i) pinMode(LEDS_ARRAY_PIN[i], OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZ_PIN, OUTPUT);
  pinMode(PIR_PIN, INPUT);
  pinMode(13, OUTPUT);
  
  pinMode(TLT_PIN, INPUT);
  digitalWrite(TLT_PIN, HIGH);   // turn on the built in pull-up resistor
  pinMode(TLT_PIN, OUTPUT);
  
  digitalWrite(13, LOW);
  // initialization message
  leds_array_blink();
  digitalWrite(LED_PIN, HIGH);
  Serial.begin(9600);
  Serial.println("# SensorsCalibration");
  Serial.println("# Francisco Zamora-Martinez (2016)");
  digitalWrite(LED_PIN, LOW);
  // initialization delay
  delay(500);
  // test led and buzzer
  blink(); blink(); buzz(); buzz();
  Vcc = SensorUtils::calibrateVcc();
} // end SETUP

void loop() {
  leds_array_blink();
  blink();
  float x_acc = AccelerometerUtils::convertToG(analogRead(ACC_X_PIN));
  float y_acc = AccelerometerUtils::convertToG(analogRead(ACC_Y_PIN));
  float z_acc = AccelerometerUtils::convertToG(analogRead(ACC_Z_PIN));
  float temp  = TemperatureUtils::convertToCelsius(analogRead(TEMP_PIN));
  int pir = digitalRead(PIR_PIN);
  int tilt = digitalRead(TLT_PIN);
  Serial.print(Vcc); Serial.print(" ");
  Serial.print(x_acc); Serial.print(" ");
  Serial.print(y_acc); Serial.print(" ");
  Serial.print(z_acc); Serial.print(" ");
  Serial.print(temp); Serial.print(" ");
  Serial.print(pir); Serial.print(" ");
  Serial.println(tilt);
  /*
    if (pir > 0) digitalWrite(BUZ_PIN, HIGH);
    else digitalWrite(BUZ_PIN, LOW);
  */
  delay(PERIOD_SLEEP);
}
