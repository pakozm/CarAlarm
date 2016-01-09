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
#ifndef TILT_SENSOR_H
#define TILT_SENSOR_H
#include "Arduino.h"
#include "AlarmSensor.h"

class TiltSensor : public AlarmSensorWithTimer {
public:
  TiltSensor(int pin) : pin(pin), count(0) {}
  
  virtual void setup() {
    pinMode(pin, INPUT);
    digitalWrite(pin, HIGH);   // turn on the built in pull-up resistor
    pinMode(pin, OUTPUT);
    delay(50);
    int val = digitalRead(pin);
    if (val == HIGH) broken = true;
    else broken = false;
    reset();
  }
  
  virtual bool checkActivity() {
    return !broken && active;
  }
  
  virtual void reset() {
    active = false;
    count  = 0;
  }
  
  virtual const char * const getName() { return "Tilt"; }
  
private:
  int pin, count;
  bool active, broken;
  static const int COUNT_THRESHOLD=10;

  virtual bool timerStep() {
    int val = digitalRead(pin);
    if (val == HIGH) ++count;
    else count = 0;
#ifdef DEBUG
    Serial.print("Tilt: count= ");
    Serial.println(count);
#endif
    active |= (count >= COUNT_THRESHOLD);
    // true forces to register again the timer
    return true;
  }
};

#endif // TILT_SENSOR_H
