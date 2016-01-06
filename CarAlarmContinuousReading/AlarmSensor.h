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
#ifndef ALARM_SENSOR_H
#define ALARM_SENSOR_H
#include <TaskTimer.h>

/**
 * @brief Abstract with interface for all available sensors.
 */
class AlarmSensor {
public:
  virtual ~AlarmSensor() {}
  
  /// initializes the sensor (probably calibrating its rest values)
  virtual void setup()=0;
  
  /// checks if sensor is active or not
  virtual bool checkActivity()=0;
  
  /// resets the alarm sensor after activation
  virtual void reset()=0;
  
  /// returns a name string
  virtual const char * const getName()=0;
};

/**
 * @brief Virtual class for timer-based sensors.
 */
class AlarmSensorWithTimer : public AlarmSensor {
public:

  virtual ~AlarmSensorWithTimer() {}
  
  /// Registers the timer at the given scheduler for the given time period.
  virtual void registerTimer(TaskTimer *scheduler, time_type period) {
    this->scheduler = scheduler;
    this->period = period;
    setupTimer();
  }

  /// Cancels the timer.
  virtual void cancelTimer() {
    scheduler->cancel(timer_id);
  }

  /// Registers the timer with previous one configuration.
  virtual void setupTimer() {
    timer_id = scheduler->timer(period, timerFunc, this);
  }
  
protected:

  /// Virtual method for delegation of timer step execution.
  virtual bool timerStep()=0;
private:
  TaskTimer *scheduler;
  time_type period;
  id_type timer_id;

  /// Static method, delegates its execution on timerStep() method.
  static void timerFunc(void *ptr) {
    AlarmSensorWithTimer *self = static_cast<AlarmSensorWithTimer*>(ptr);
    if (self->timerStep()) self->registerTimer(self->scheduler, self->period);
  }
};

#endif // ALARM_SENSOR_H
