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
#define ACT_COUNT_TH 10

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
    temp_ref  = readTemperature();
    active    = false;
    count     = 0;
    act_count = 0;
  }
  virtual const char * const getName() { return "Temp"; }
  
  /**
   *  sensor values: 17C..27C
   *  real measure:  20C..37C
   */
  long readTemperature() const {
    static const long in_a  = 170, in_b  = 270;
    static const long out_a = 200, out_b = 370;
    analogRead(pin);
    delay(100);
    return map(TemperatureUtils::convertToCelsius(analogRead(pin)), in_a, in_b, out_a, out_b);
  }
  
private:
  int   pin, count, act_count;
  long  temp_ref;
  bool  active;
  static const long ALPHA    = 90; // 90%
  static const long EPSILON  = 20; // 2C

  virtual bool timerStep() {
    long cur      = readTemperature();
    long abs_diff = abs(temp_ref - cur);
    if (Serial) {
      Serial.print("Temp: ref= ");
      Serial.println(temp_ref);
      Serial.print("      cur= ");
      Serial.println(cur);
    }
    temp_ref = (ALPHA*temp_ref)/100 + ((100-ALPHA)*cur)/100;
    if ((count>COUNT_TH) && (abs_diff > EPSILON)) ++act_count;
    active |= act_count > ACT_COUNT_TH;
    count++;
    // true forces to register again the timer
    return true;
  }
};

#undef COUNT_TH
#undef ACT_COUNT_TH

#endif // TEMPERATURE_SENSOR_H

