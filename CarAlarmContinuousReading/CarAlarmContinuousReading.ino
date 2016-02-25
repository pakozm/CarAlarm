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
#include <TaskTimer.h>
#include <SensorTransformations.h>

// #define DEBUG

#include "AlarmSensor.h"
#include "AccelerometerSensor.h"
#include "PIRSensor.h"
#include "TemperatureSensor.h"

const byte VERSION = 07; // firmware version divided by 10 e,g 16 = V1.6
const long REF_CAL = 1120000L;
// WARNING: should be PERIOD_SLEEP >= 100
const unsigned long PERIOD_SLEEP =  1000; // 1000 ms
const unsigned long BLINK_DELAY  = 10000; // 10 seconds

const unsigned long CALIBRATION_DELAY  = 900000; // 15 minutes
const unsigned long TEMPERATURE_PERIOD =   2000; //  2 second

#ifdef DEBUG
const unsigned long START_SLEEP    =  1000; // 1 seconds
const unsigned long REARM_DELAY    =  4000; // 4 seconds
const unsigned long ALARM_DELAY    =  1000; // 1 seconds
const unsigned long ALARM_DURATION =  6000; // 6 seconds
#else
const unsigned long START_SLEEP    =  60000; // 60 seconds
const unsigned long REARM_DELAY    = 900000; // 15 minutes
const unsigned long ALARM_DELAY    =  10000; // 10 seconds
const unsigned long ALARM_DURATION =  60000; // 60 seconds
#endif

const int BATTERY_OK_REPETITIONS    = 2;
const int BATTERY_OK_DURATION       = 50; // 50 ms

const int BATTERY_ALERT_REPETITIONS = 3;
const int BATTERY_ALERT_DURATION    = 500; // 0.5 seconds

const int BATTERY_ERROR_REPETITIONS = 10;
const int BATTERY_ERROR_DURATION    = 100; // 100 ms

const long VCC_ALERT = 4600; // mili-volts
const long VCC_ERROR = 4300; // mili-volts

const int NUM_SENSORS = 3;
const int MAX_TASKS = 10; // maximum number of tasks for Scheduler

// digital pins connection
const int LED_PIN = 9;
const int SRN_PIN = 10;
const int BUZ_PIN = 11;
const int PIR_PIN = 12;

// analogic pins connection
const int ACC_POT_PIN = 5;
const int ACC_X_PIN   = 4;
const int ACC_Y_PIN   = 3;
const int ACC_Z_PIN   = 2;
const int TMP_POT_PIN = 1;
const int TMP_PIN     = 0;

// default thresholds
const float DEF_ACC_TH = 10.0f;
const long DEF_TMP_EPS = 20;   // 2C

long Vcc; // in mili-volts
TaskTimerWithHeap<MAX_TASKS> scheduler;
id_type alarm_task, blink_task, calibration_task;

///////////////////////////////////////////////////////////////////////////

AccelerometerSensor acc_sensor(ACC_X_PIN, ACC_Y_PIN, ACC_Z_PIN, DEF_ACC_TH);
PIRSensor pir_sensor(PIR_PIN);
TemperatureSensor temp_sensor(TMP_PIN, DEF_TMP_EPS);

AlarmSensor *sensors[NUM_SENSORS] = { 
  &acc_sensor,
  &pir_sensor,
  &temp_sensor };

template<typename T> void print(const T &obj);
template<typename T> void println(const T &obj);

template<typename T>
void print(const T &obj) {
  if (Serial) Serial.print(obj);
}

template<typename T>
void println(const T &obj) {
  if (Serial) Serial.println(obj);
}

void print_seconds(const char *prefix, unsigned long ms) {
  if (Serial) {
    Serial.print(prefix);
    Serial.print(" ");
    Serial.print(ms/1000.0f);
    Serial.println(" seconds");
  }
}

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

void siren_on() {
  digitalWrite(SRN_PIN, HIGH);
}

void siren_off() {
  digitalWrite(SRN_PIN, LOW);
}

/////////////////////////////////////////////////////////////////////////////

void blink(unsigned long ms=100, unsigned long post_ms=200) {
  led_on(); 
  sleep(ms);
  led_off(); 
  sleep(post_ms);
}

void buzz(unsigned long ms=100, unsigned long post_ms=200) {
  buzzer_on(); 
  sleep(ms);
  buzzer_off(); 
  sleep(post_ms);
}

void alarmAlert() {
  led_on();
  print_seconds("ALARM ON: delaying", ALARM_DELAY);
  sleep(ALARM_DELAY);
  print_seconds("          buzzing during", ALARM_DURATION);
  led_off();

  unsigned long t0 = millis();
  siren_on();
  while(millis() - t0 < ALARM_DURATION) {
    led_on();
    sleep(1000);
    led_off();
    sleep(1000);
  }
  siren_off();
  
  println("ALARM OFF");
}

bool check_failure_Vcc() {
  if (Vcc < VCC_ERROR) {
    for (int i=0; i<BATTERY_ERROR_REPETITIONS; ++i) {
#ifdef DEBUG
      buzz(BATTERY_ERROR_DURATION);
#else
      siren_on(); sleep(50); siren_off(); sleep(100);
#endif
    }
    return true;
  }
  else if (Vcc < VCC_ALERT) {
    for (int i=0; i<BATTERY_ALERT_REPETITIONS; ++i) {
#ifdef DEBUG
      blink(100, 0);
      buzz(BATTERY_ALERT_DURATION, 0);
#else      
      siren_on(); sleep(50); siren_off(); sleep(100);
#endif
    }
  }
  else {
    // indicates correct function by buzzing and blinking
    for (int i=0; i<BATTERY_OK_REPETITIONS; ++i) {
#ifdef DEBUG
      blink(100, 0);
      buzz(BATTERY_OK_DURATION, 0);
#else
      siren_on(); sleep(50); siren_off(); sleep(100);
#endif 
    }
  }
  return false;
}

void blink_and_repeat(void *) {
  blink();
  blink_task = scheduler.timer(BLINK_DELAY, blink_and_repeat);
}

void rearm_alarm(void *) {
  for (int i=0; i<NUM_SENSORS; ++i) {
    sensors[i]->reset();
  }
  alarm_task = scheduler.timer(PERIOD_SLEEP, alarm_check);
}

void alarm_check(void *)
{
  int i, activity_detected=0;

  for (i=0; i<NUM_SENSORS; ++i) {
    if (sensors[i]->checkActivity()) {
      print("Activity at sensor: ");
      println(sensors[i]->getName());
      activity_detected = 1;
    }
  }
  if (activity_detected) {
    alarmAlert();
    print_seconds("ALARM rearm in", REARM_DELAY);
    alarm_task = scheduler.timer(REARM_DELAY, rearm_alarm);
  }
  else {
    /*
      println("No activity detected");
      print_seconds("ALARM check in", PERIOD_SLEEP);
    */
    alarm_task = scheduler.timer(PERIOD_SLEEP, alarm_check);
  }
}

void calibrate_timer(void *) {
  long Vcc = SensorUtils::calibrateVcc(REF_CAL);
  print("Vcc= "); 
  println(Vcc);
  calibration_task = scheduler.timer(CALIBRATION_DELAY, calibrate_timer);
}

long readPotentiometer(int pin) {
  analogRead(pin);
  delay(50);
  return analogRead(pin);
}

///////////////////////////////////////////////////////////////////////////////

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZ_PIN, OUTPUT);
  pinMode(SRN_PIN, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  digitalWrite(LED_PIN, HIGH);
  // initialization message
  Serial.begin(9600);
  delay(100);
  while (!Serial); // wait for Leonardo
  print("CarAlarmContinuousReading V");
  println(VERSION*0.1f);
  println("Francisco Zamora-Martinez (2016)");
  digitalWrite(LED_PIN, LOW);

  Vcc = SensorUtils::calibrateVcc(REF_CAL);
  print("Vcc=     "); 
  println(Vcc);
  if (check_failure_Vcc()) return;

  long acc_raw = readPotentiometer(TMP_POT_PIN);

  long tmp_raw = readPotentiometer(ACC_POT_PIN);

  float acc_th = (acc_raw/1023.0f)*20.0f;
  long tmp_eps = map(tmp_raw, 0, 1023, 0, 80);

  acc_sensor.setThreshold(acc_th);
  temp_sensor.setEpsilon(tmp_eps);

  print("ACC RAW= "); 
  println(acc_raw);
  print("TMP RAW= "); 
  println(tmp_raw);

  print("ACC TH=  "); 
  println(acc_th);
  print("TMP EPS= "); 
  println(tmp_eps);

  print_seconds("START.....wait ", START_SLEEP);
  sleep(START_SLEEP);
  blink();

  // initialize all installed sensors
  for (int i=0; i<NUM_SENSORS; ++i) {
    sensors[i]->setup();
  }

  // register timer-based sensors
  temp_sensor.registerTimer(&scheduler, TEMPERATURE_PERIOD);

  // schedule all required tasks
  blink_task = scheduler.timer(BLINK_DELAY, blink_and_repeat);
  alarm_task = scheduler.timer(PERIOD_SLEEP, alarm_check);
  calibration_task = scheduler.timer(CALIBRATION_DELAY, calibrate_timer);

} // end SETUP

void loop() {
  if (Vcc < VCC_ERROR) {
    println("VCC_ERROR");
    sleep(100000);
  }
  else if (scheduler.pollWaiting() == ALL_IDLE) {
    println("ALL_IDLE");
    sleep(100000);
  }
}

