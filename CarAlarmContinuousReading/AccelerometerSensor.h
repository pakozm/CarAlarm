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
#ifndef ACC_SENSOR_H
#define ACC_SENSOR_H
#include "Arduino.h"
#include "AlarmSensor.h"
#include "AccelerometerUtils.h"

class AccelerometerSensor : public AlarmSensor {
public:
  AccelerometerSensor(int x_pin, int y_pin, int z_pin, float threshold) :
    threshold2(threshold*threshold) {
    pins[0] = x_pin;
    pins[1] = y_pin;
    pins[2] = z_pin;
    zeros(m_refs);
    zeros(s_refs);
  }
  
  void setThreshold(float th) {
    threshold2 = th * th;
  }
  
  virtual void setup() {
    float samples[SETUP_NUM_SAMPLES][3];
    for (int i=0; i<SETUP_NUM_SAMPLES; ++i) {
      readData(samples[i]);
      axpy(m_refs, 1.0f, samples[i]);
      delay(SETUP_SAMPLE_DELAY);
    }
    scal(m_refs, 1.0f/SETUP_NUM_SAMPLES);
    float dif[3];
    for (int i=0; i<SETUP_NUM_SAMPLES; ++i) {
      copy(dif, samples[i]);
      axpy(dif, -1.0f, m_refs);
      square(dif);
      axpy(s_refs, 1.0f, dif);
    }
    scal(s_refs, 1.0f/(SETUP_NUM_SAMPLES-1));
    squareRoot(s_refs);
    
    if (Serial) {
      Serial.print("ACC Reference: mu=    ");
      println(m_refs);
      Serial.print("               sigma= ");
      println(s_refs);
    }

    copy(high_refs, m_refs);
    copy(low_refs, m_refs);
    axpy(high_refs, 2.0f, s_refs);
    axpy(low_refs, -2.0f, s_refs);
  }
  
  virtual bool checkActivity() {
    float vec[3];
    readData(vec);
    bool activity = false;
    for (int i=0; i<3; ++i) {
      if (vec[i] < low_refs[i] || vec[i] > high_refs[i]) {
        activity=true;
      }
    }
    if (Serial) {
      if (activity) {
        Serial.print("ACC: vec= ");
        println(vec);
      }
    }
    axpy(vec, -1.0f, m_refs);
    square(vec);
    float s = sum(vec);
    if (Serial) {
      if (s > threshold2) {
        Serial.print("     sum= ");
        Serial.println(s);
      }
    }
    return s > threshold2 && activity;
  }
  
  virtual void reset() {}
  
  virtual const char * const getName() { return "ACC"; }

  long readAccX() const {
    static const long min=-20, max=19;
    return map(AccelerometerUtils::convertToG(analogRead(pins[0])), min, max, -100, 100);
  }

  long readAccY() const {
    static const long min=-20, max=20;
    return map(AccelerometerUtils::convertToG(analogRead(pins[1])), min, max, -100, 100);
  }

  long readAccZ() const {
    static const long min=-17, max=23;
    return map(AccelerometerUtils::convertToG(analogRead(pins[2])), min, max, -100, 100);
  }
  
private:
  int pins[3];
  float m_refs[3], s_refs[3], low_refs[3], high_refs[3];
  float threshold2;
  static const int SETUP_NUM_SAMPLES=30;
  static const unsigned long SETUP_SAMPLE_DELAY=50;
  
  template<typename T>
  void print(const T *vec) {
    for (int i=0; i<3; ++i) {
      Serial.print(vec[i]);
      Serial.print(" ");
    }
  }
  
  template<typename T>
  void println(const T *vec) {
    for (int i=0; i<3; ++i) {
      Serial.print(vec[i]);
      Serial.print(" ");
    }
    Serial.println("");
  }

  void readData(float *vec) {
    vec[0] = readAccX();
    vec[1] = readAccY();
    vec[2] = readAccZ();
  }

  void zeros(float *vec) {
    for (int j=0; j<3; ++j) vec[j] = 0.0f;
  }

  void copy(float *dst, const float *src) {
    for (int j=0; j<3; ++j) dst[j] = src[j];
  }

  float sum(float *vec) {
    float r = 0.0f;
    for (int j=0; j<3; ++j) r += vec[j];
    return r;
  }

  void axpy(float *dst, float a, const float *src) {
    for (int j=0; j<3; ++j) dst[j] += a*src[j];
  }

  void scal(float *dst, float s) {
    for (int j=0; j<3; ++j) dst[j] *= s;
  }

  void square(float *dst) {
    for (int j=0; j<3; ++j) dst[j] *= dst[j];
  }

  void squareRoot(float *dst) {
    for (int j=0; j<3; ++j) {
      if (dst[j] != 0.0f) dst[j] = sqrtf(dst[j]);
    }
  }
};

#endif // ACC_SENSOR_H
