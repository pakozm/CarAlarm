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
ISR(WDT_vect) { 
  Sleepy::watchdogEvent(); 
} // Setup for low power waiting
#include <SensorTransformations.h>

const unsigned long PERIOD_SLEEP = 100; // 100 ms

// digital pins connection
const int BUZ_PIN = 11;
const int LED_PIN = 12;
const int PIR_PIN = 2;

// analogic pins connection
const int ACC_X_PIN = 2;
const int ACC_Y_PIN = 1;
const int ACC_Z_PIN = 0;
const int TMP_PIN  = 5;

long Vcc; // in mili-volts

long minX=-1, maxX=1;
/**
 * @note x:[-0.23,0.16]
 */
long readAccX()  {
  long v = AccelerometerUtils::convertToG(analogRead(ACC_X_PIN));
  if (v < minX) minX = v;
  if (v > maxX) maxX = v;
  return map(v, minX, maxX, -100, 100);
}

long minY=-1, maxY=1;
/**
 * @note y:[-0.22,0.17]
 */
long readAccY()  {
  long v = AccelerometerUtils::convertToG(analogRead(ACC_Y_PIN));
  if (v < minY) minY = v;
  if (v > maxY) maxY = v;
  return map(v, minY, maxY, -100, 100);
}

long minZ=-1, maxZ=1;
/**
 * @note z:[-0.19,0.22]
 */
long readAccZ()  {
  long v = AccelerometerUtils::convertToG(analogRead(ACC_Z_PIN));
  if (v < minZ) minZ = v;
  if (v > maxZ) maxZ = v;
  return map(v, minZ, maxZ, -100, 100);
}


long readTemperature()  {
  static const long a  = 200, b  = 370;
  static const long ah = 170, bh = 270;
  analogRead(TMP_PIN);
  delay(100);
  Serial.println(TemperatureUtils::convertToCelsius(analogRead(TMP_PIN)));
  return map(TemperatureUtils::convertToCelsius(analogRead(TMP_PIN)), ah, bh, a, b);
}


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
  led_on(); 
  delay(ms);
  led_off(); 
  delay(post_ms);
}

void buzz(unsigned long ms=100, unsigned long post_ms=200) {
  buzzer_on(); 
  delay(ms);
  buzzer_off(); 
  delay(post_ms);
}

///////////////////////////////////////////////////////////////////////////////

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZ_PIN, OUTPUT);
  pinMode(PIR_PIN, INPUT);
  pinMode(13, OUTPUT);

  digitalWrite(13, LOW);
  // initialization message
  digitalWrite(LED_PIN, HIGH);
  Serial.begin(9600);
  Serial.println("# SensorsCalibration");
  Serial.println("# Francisco Zamora-Martinez (2016)");
  digitalWrite(LED_PIN, LOW);
  // initialization delay
  delay(500);
  // test led and buzzer
  blink(); 
  blink(); 
  buzz(); 
  buzz();
  Vcc = SensorUtils::calibrateVcc();
} // end SETUP

float pow2(float x) { 
  return x*x; 
}

void loop() {
  blink();
  analogRead(ACC_X_PIN); 
  delay(50);
  long x_acc = readAccX();
  long y_acc = readAccY();
  long z_acc = readAccZ();
  long sum = pow2(0.0f - x_acc) + pow2(0.0f - y_acc) + pow2(1.0f - z_acc);
  long temp  = readTemperature();
  int pir = digitalRead(PIR_PIN);
  Serial.print(Vcc); 
  Serial.print(" ");
  Serial.print(x_acc); 
  Serial.print(" ");
  Serial.print(y_acc); 
  Serial.print(" ");
  Serial.print(z_acc); 
  Serial.print(" ");
  Serial.print(sum); 
  Serial.print(" ");
  Serial.print(temp); 
  Serial.print(" ");
  Serial.println(pir);
  
  Serial.print("     ");
  Serial.print(minX); Serial.print(" ");
  Serial.print(maxX); Serial.print(" ");
  Serial.print(minY); Serial.print(" ");
  Serial.print(maxY); Serial.print(" ");
  Serial.print(minZ); Serial.print(" ");
  Serial.print(maxZ); Serial.println("");  
  
  /*
    if (pir > 0) digitalWrite(BUZ_PIN, HIGH);
   else digitalWrite(BUZ_PIN, LOW);
   */
  delay(PERIOD_SLEEP);
}

