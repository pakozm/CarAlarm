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

/*******************************************************************************
 * Summary:
 * 
 * This alarm uses different sensors to detect activity inside the car. When any
 * of them sense activity, the alarm will start making noise for a particular
 * duration.
 * 
 * The alarm has a START_DELAY time which allow the user to arm the alarm and
 * leave the car. Once this delay passes, the alarm executes a routine every
 * PERIOD_SLEEP looking for activity in the sensors. When activity is sensed, a
 * delay of ALARM_DELAY is used to allow manual disconnection of the alarm. After
 * this time the alarm starts making noise during ALARM_DURATION time. After this
 * time the alarm waits REARM_DELAY before rearming alarm again.
 * 
 * 
 * Setup procedure: depending on alarm battery level the starting procedure
 * changes, indicating if everything is ok to work, if the battery needs to be
 * replaced/charged, or if battery is so low that the system won't run:
 * 
 * 1. Correct function is advertised by BATTERY_OK_REPETITIONS short buzzes.
 * 
 * 2. Low battery is advertised by BATTERY_ALERT_REPETITIONS long buzzes, but the
 * alarm stills running.
 * 
 * 3. Very low battery is advertised by BATTERY_ERROR_REPETITIONS short buzzes
 * and the system won't run.
 * 
 * 
 * Change Log:
 * v0.8 2016/03/13 Major code refactoring
 * v0.7 2016/02/25 Improvements in alarm reset and TaskTimer run method.
 * v0.6 2016/02/23 Fixing REF_CAL for EcoDuino PCB shield.
 * v0.5 2016/01/11 Using integer math for sensors.
 * v0.4 2016/01/03 Using a timer scheduler.
 * v0.3 2016/01/02 Added battery level indications at start.
 * v0.2 2016/01/02 Added readVcc() routine.
 * v0.1 2016/01/01 First draft: buzzer, PIR and accelerometer.
 *******************************************************************************/
#define CAR_ALARM_INO

#include <JeeLib.h>
ISR(WDT_vect) { 
  Sleepy::watchdogEvent(); 
} // Setup for low power waiting
#include <AlarmUtils.h>
#include <TaskTimer.h>
#include <SensorTransformations.h>

const byte VERSION = 8; // firmware version divided by 10 e,g 16 = V1.6
const unsigned long ALARM_DELAY_MODE_ON = 20000; // 20 seconds

const int MAX_TASKS = 10; // maximum number of tasks for Scheduler

TaskTimerWithHeap<MAX_TASKS> scheduler;

///////////////////////////////////////////////////////////////////////////////

void setup()
{
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  setupAlarmPins();
  
  Serial.begin(9600);
  delay(100);
  while (!Serial); // wait for Leonardo
  Serial.print("CarAlarmContinuousReading V");
  Serial.println(VERSION*0.1f);
  Serial.println("Francisco Zamora-Martinez (2016)");
  setupAlarm(&scheduler, ALARM_DELAY_MODE_ON);
} // end SETUP

void loop() {
  if (scheduler.pollWaiting() == ALL_IDLE) {
    Serial.println("ALL_IDLE");
    sleep(100000);
  }
}
