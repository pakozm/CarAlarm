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
#ifndef TEMPERATURE_SENSOR_H
#define TEMPERATURE_SENSOR_H
#include "Arduino.h"
#include "AlarmSensor.h"
#include "TemperatureUtils.h"

// 10 samples lag to compare temperatures and compute gradient
#define COUNT_TH 10

class TemperatureSensor : public AlarmSensorWithTimer {
public:
  TemperatureSensor(int pin) : pin(pin) {}

  virtual void setup() {
    pinMode(pin, INPUT);
    reset();
  }
  
  virtual bool checkActivity() {
    return active;
  }
  
  virtual void reset() {
    temp_ref = readRawTemperature();
    for (int i=0; i<COUNT_TH; ++i) temps[i] = temp_ref;
    active   = false;
    count    = 0;
  }
  virtual const char * const getName() { return "Temp"; }
  
  float readTemperature() const {
    return temps[(count+1)%COUNT_TH];
  }
  
  float readRawTemperature() const {
    analogRead(pin);
    delay(50);
    return TemperatureUtils::convertToCelsius(analogRead(pin));
  }
  
private:
  int   pin, count;
  float temp_ref, temps[COUNT_TH];
  bool  active;
  static const float ALPHA    = 0.9;
  static const float EPSILON  = 1.0;

  virtual bool timerStep() {
    float raw      = readRawTemperature();
    float cur      = readTemperature();
    float abs_diff = fabsf(temp_ref - cur);
#ifdef DEBUG
    Serial.print("Temp: ref= ");
    Serial.println(temp_ref);
    Serial.print("      cur= ");
    Serial.println(cur);
    Serial.print("      raw= ");
    Serial.println(raw);
#endif
    temp_ref = ALPHA*temp_ref + (1-ALPHA)*raw;
    active |= (count>COUNT_TH) && (abs_diff > EPSILON);
    count++;
    temps[count%COUNT_TH] = temp_ref;
    // true forces to register again the timer
    return true;
  }
};

#undef COUNT_TH

#endif // TEMPERATURE_SENSOR_H
