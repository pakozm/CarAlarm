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

#include <TaskTimer.h>
#include <SensorTransformations.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>

#define DEBUG

#include "AlarmUtils.h"
#include "AlarmSensor.h"
#include "AccelerometerSensor.h"
#include "PIRSensor.h"
#include "TemperatureSensor.h"

const long REF_CAL = 1120000L;
// WARNING: should be PERIOD_SLEEP >= 100
const unsigned long PERIOD_SLEEP =  1000; // 1000 ms
const unsigned long BLINK_DELAY  = 10000; // 10 seconds

const unsigned long CALIBRATION_DELAY  = 900000; // 15 minutes
const unsigned long TEMPERATURE_PERIOD =   2000; //  2 second

#ifdef DEBUG
const unsigned long START_SLEEP    =  1000; // 1 seconds
const unsigned long REARM_DELAY    =  4000; // 4 seconds
const unsigned long ALARM_DURATION =  6000; // 6 seconds
unsigned long ALARM_DELAY          =  1000; // 1 seconds
#else
const unsigned long START_SLEEP    = 120000; //  2 minutes
const unsigned long REARM_DELAY    = 900000; // 15 minutes
const unsigned long ALARM_DURATION =  60000; // 60 seconds
unsigned long ALARM_DELAY          =  20000; // 20 seconds
#endif

const int CANCEL_ALARM_REPETITIONS    = 2;
const int CANCEL_ALARM_DURATION       = 50; // 50 ms

const int BATTERY_OK_REPETITIONS    = 1;
const int BATTERY_OK_DURATION       = 50; // 50 ms

const int BATTERY_ALERT_REPETITIONS = 5;
const int BATTERY_ALERT_DURATION    = 50; // 50 ms

const int BATTERY_ERROR_REPETITIONS = 10;
const int BATTERY_ERROR_DURATION    = 50; // 50 ms

const long VCC_ALERT = 4600; // mili-volts
const long VCC_ERROR = 4300; // mili-volts

const int NUM_SENSORS = 3;

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
TaskTimer *_scheduler;
id_type alarm_task, blink_task, calibration_task, setup_task, alarm_alert_task;
bool started = false, alerting = false;
unsigned long t0;

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

void blink_and_repeat(void *);
void rearm_alarm(void *);

void adc_on()
{
  // activate ADC
  power_adc_enable();
  ADCSRA |= (1 << ADEN);
  delay(10);
}

void adc_off()
{
  // Disable ADC
  ADCSRA &= ~(1 << ADEN);
  power_adc_disable();
}

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

void blink(unsigned long ms, unsigned long post_ms) {
  led_on(); 
  delay(ms);
  led_off(); 
  delay(post_ms);
}

void buzz(unsigned long ms, unsigned long post_ms) {
  buzzer_on(); 
  delay(ms);
  buzzer_off(); 
  delay(post_ms);
}

bool started_alert = false;
void alarmAlertTask(void *)
{
  print_seconds("          buzzing during", ALARM_DURATION);
  led_off();
  if (!started_alert) {
    started_alert = true;
    print_seconds("          buzzing during", ALARM_DURATION);
    siren_on();
  }
  if (millis() - t0 < ALARM_DURATION) {
#ifdef DEBUG
    buzz();
#endif
    blink(100,100);
    blink(100,0);
    alarm_alert_task = _scheduler->timer(1800, alarmAlertTask);
  }
  else {
    siren_off();
    alerting = false;
    println("ALARM OFF");
    print_seconds("ALARM rearm in", REARM_DELAY);
    blink_task = _scheduler->timer(BLINK_DELAY, blink_and_repeat);
    alarm_task = _scheduler->timer(REARM_DELAY, rearm_alarm);
    temp_sensor.registerTimer(_scheduler, TEMPERATURE_PERIOD);
  }
}

void alarmAlert() {
  led_on();
  print_seconds("ALARM ON: delaying", ALARM_DELAY);
  t0 = millis() + ALARM_DELAY;
  alerting = true;
  started_alert = false;
  led_on();
  alarm_alert_task = _scheduler->timer(ALARM_DELAY, alarmAlertTask);
  _scheduler->cancel(blink_task);
  _scheduler->cancel(alarm_task);
  temp_sensor.cancelTimer();
}

bool check_failure_Vcc() {
  if (Vcc < VCC_ERROR) {
    for (int i=0; i<BATTERY_ERROR_REPETITIONS; ++i) {
#ifdef DEBUG
      blink(100, 5);
      buzz(BATTERY_ERROR_DURATION);
#else
      siren_on(); delay(BATTERY_ERROR_DURATION); siren_off(); delay(100);
#endif
    }
    return true;
  }
  else if (Vcc < VCC_ALERT) {
    for (int i=0; i<BATTERY_ALERT_REPETITIONS; ++i) {
#ifdef DEBUG
      blink(100, 5);
      buzz(BATTERY_ALERT_DURATION, 0);
#else      
      siren_on(); delay(BATTERY_ALERT_DURATION); siren_off(); delay(100);
#endif
    }
  }
  else {
    // indicates correct function by buzzing and blinking
    for (int i=0; i<BATTERY_OK_REPETITIONS; ++i) {
#ifdef DEBUG
      blink(100, 5);
      buzz(BATTERY_OK_DURATION, 0);
#else
      siren_on(); delay(BATTERY_OK_DURATION); siren_off(); delay(100);
#endif 
    }
  }
  return false;
}

void blink_and_repeat(void *) {
  blink();
  blink_task = _scheduler->timer(BLINK_DELAY, blink_and_repeat);
}

void alarm_check(void *);

void rearm_alarm(void *) {
  adc_on();

  for (int i=0; i<NUM_SENSORS; ++i) {
    sensors[i]->reset();
  }
  alarm_task = _scheduler->timer(PERIOD_SLEEP, alarm_check);
  
  adc_off();
}

void alarm_check(void *)
{
  int i, activity_detected=0;

  adc_on();
  
  for (i=0; i<NUM_SENSORS; ++i) {
    if (sensors[i]->checkActivity()) {
      print("Activity at sensor: ");
      println(sensors[i]->getName());
      activity_detected = 1;
    }
  }
  if (activity_detected) {
    alarmAlert();
  }
  else {
    /*
      println("No activity detected");
      print_seconds("ALARM check in", PERIOD_SLEEP);
    */
    alarm_task = _scheduler->timer(PERIOD_SLEEP, alarm_check);
  }

  adc_off();
}

void calibrate_timer(void *) {
  long Vcc = SensorUtils::calibrateVcc(REF_CAL);
  print("Vcc= "); 
  println(Vcc);
  calibration_task = _scheduler->timer(CALIBRATION_DELAY, calibrate_timer);
}

long readPotentiometer(int pin) {
  analogRead(pin);
  delay(50);
  return analogRead(pin);
}

///////////////////////////////////////////////////////////////////////////////

void setupTask(void *)
{
  adc_on();
  
  started = true;
  blink();
  // initialize all installed sensors
  for (int i=0; i<NUM_SENSORS; ++i) {
    sensors[i]->setup();
  }
    // register timer-based sensors
  temp_sensor.registerTimer(_scheduler, TEMPERATURE_PERIOD);
  // schedule all required tasks
  blink_task = _scheduler->timer(BLINK_DELAY, blink_and_repeat);
  alarm_task = _scheduler->timer(PERIOD_SLEEP, alarm_check);
  calibration_task = _scheduler->timer(CALIBRATION_DELAY, calibrate_timer);
  
  adc_off();
}

///////////////////////////////////////////////////////////////////////////////

void setupAlarmPins()
{
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZ_PIN, OUTPUT);
  pinMode(SRN_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(50);
  digitalWrite(LED_PIN, LOW);
}

void cancelAlarmPins()
{
  pinMode(LED_PIN, INPUT);
  pinMode(BUZ_PIN, INPUT);
  pinMode(SRN_PIN, INPUT);
}

void setupAlarm(TaskTimer *sched_arg, unsigned long alarm_delay)
{
  adc_on();
  
  _scheduler = sched_arg;
  ALARM_DELAY = alarm_delay;
  started = false;
  // initialization message

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
  setup_task = _scheduler->timer(START_SLEEP, setupTask);
  
  adc_off();
} // end SETUP

void cancelAlarm() {
  if (started) {
    if (alerting) {
      led_off();
      _scheduler->cancel(alarm_alert_task);
    }
    else {
      _scheduler->cancel(blink_task);
      _scheduler->cancel(alarm_task);
      temp_sensor.cancelTimer();
    }
    _scheduler->cancel(calibration_task);
  }
  else {
    _scheduler->cancel(setup_task);
  }
  started = false;
  for (int i=0; i<CANCEL_ALARM_REPETITIONS; ++i) {
#ifdef DEBUG
      blink(100, 5);
      buzz(CANCEL_ALARM_DURATION, 0);
#else
      siren_on(); delay(CANCEL_ALARM_DURATION); siren_off(); delay(100);
#endif 
  }
}
