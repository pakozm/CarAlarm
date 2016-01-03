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
  Summary:
  
  This alarm uses different sensors to detect activity inside the car. When any
  of them sense activity, the alarm will start making noise for a particular
  duration.
  
  The alarm has START_DELAY which allow the user to put it on and leave the car.
  Once this delay passes, the alarm executes a routine every PERIOD_SLEEP
  looking for activity in the sensors. When activity is sensed, a delay of
  ALARM_DELAY is used to allow manual disconnection of the alarm. After this
  time the alarm starts making noise during ALARM_DURATION time. After this
  time the alarm waits REARM_DELAY before rearming alarm sensors.

  
  Starting procedure: depending on alarm battery level the starting procedure
  changes, indicating if everything is ok to work, if the battery needs to be
  replaced/charged, or if battery is so low that the system won't run:
  
  1. Correct function is advertised by BATTERY_OK_REPETITIONS short buzzes.
  
  2. Low battery is advertised by BATTERY_ALERT_REPETITIONS long buzzes, but the
     alarm stills running.

  3. Very low battery is advertised by BATTERY_ERROR_REPETITIONS short buzzes
     and the system won't run.
  
  
  Change Log:
  v0.4 2016/01/03 Using a timer scheduler.
  v0.3 2016/01/02 Added battery level indications at start.
  v0.2 2016/01/02 Added readVcc() routine.
  v0.1 2016/01/01 First draft: buzzer, PIR and accelerometer.
*******************************************************************************/

/*
  TODO: add interrupt-based controller for PIR sensor
*/

#include <JeeLib.h>
ISR(WDT_vect) { Sleepy::watchdogEvent(); } // Setup for low power waiting

#include <TaskTimer.h>

#define DEBUG

const byte VERSION = 04; // firmware version divided by 10 e,g 16 = V1.6
// WARNING: should be PERIOD_SLEEP >= 100
const unsigned long PERIOD_SLEEP =   100; // 100 ms
const unsigned long BLINK_DELAY  = 10000; // 10 s

#ifdef DEBUG
const unsigned long START_SLEEP    =  1000; // 1 s
const unsigned long REARM_DELAY    =  4000; // 4 s
const unsigned long ALARM_DELAY    =  1000; // 1 s
const unsigned long ALARM_DURATION =  6000; // 6 seconds
#else
const unsigned long START_SLEEP    = 60000; // 60 s
const unsigned long REARM_DELAY    = 60000; // 60 s
const unsigned long ALARM_DELAY    = 10000; // 10 s
const unsigned long ALARM_DURATION = 60000; // 60 seconds
#endif

const int BATTERY_OK_REPETITIONS    = 2;
const int BATTERY_ALERT_REPETITIONS = 3;
const int BATTERY_ERROR_REPETITIONS = 10;

const long VCC_ALERT = 4000; // mili-volts
const long VCC_ERROR = 3000; // mili-volts

const int NUM_SENSORS = 2;
const int MAX_TASKS = 4; // maximum number of tasks for Scheduler

// digital pins connection
const int PIR_PIN = 2;
const int LED_PIN = 13;
const int BUZ_PIN = 12;

// analogic pins connection
const int ACC_X_PIN = 0;
const int ACC_Y_PIN = 1;
const int ACC_Z_PIN = 2;

// accelerometer threshold
const float ACC_TH = 10.0f;

long Vcc; // in mili-volts
TaskTimer<MAX_TASKS> scheduler;
id_type blink_task, alarm_task;

///////////////////////////////////////////////////////////////////////////

class AlarmSensor {
public:
virtual ~AlarmSensor() {}
// initializes the sensor (probably calibrating its rest values)
virtual void setup()=0;
// checks if sensor is active or not
virtual bool checkActivity()=0;
//
virtual const char * const getName()=0;
};

class PIRSensor : public AlarmSensor {
public:
PIRSensor(int pin) : pin(pin) { };
virtual void setup() {
pinMode(pin, INPUT);
}
virtual bool checkActivity() {
int val = digitalRead(pin);
return (val == HIGH);
}
virtual const char * const getName() { return "PIR"; }
private:
int pin, count;
};

class AccelerometerSensor : public AlarmSensor {
public:
AccelerometerSensor(int x_pin, int y_pin, int z_pin, float threshold) :
  x_pin(x_pin), y_pin(y_pin), z_pin(z_pin),
  threshold2(threshold*threshold) { };
  virtual void setup() {
    x_ref = analogRead(x_pin);
    y_ref = analogRead(y_pin);
    z_ref = analogRead(z_pin);
  }
  virtual bool checkActivity() {
    int x_val, y_val, z_val;
    x_val = analogRead(x_pin);
    y_val = analogRead(y_pin);
    z_val = analogRead(z_pin);
    float dx=x_val-x_ref, dy=y_val-y_ref, dz=z_val-z_ref;
    return (dx*dx + dy*dy + dz*dy) > threshold2;
  }
  virtual const char * const getName() { return "ACC"; }
private:
  int x_pin, y_pin, z_pin;
  int x_ref, y_ref, z_ref;
  float threshold2;
};

///////////////////////////////////////////////////////////////////////////

PIRSensor pir_sensor(PIR_PIN);
AccelerometerSensor acc_sensor(ACC_X_PIN, ACC_Y_PIN, ACC_Z_PIN, ACC_TH);

AlarmSensor *sensors[NUM_SENSORS] = { &pir_sensor, &acc_sensor };

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
  led_on(); sleep(ms);
  led_off(); sleep(post_ms);
}

void buzz(unsigned long ms=100, unsigned long post_ms=200) {
  buzzer_on(); sleep(ms);
  buzzer_off(); sleep(post_ms);
}

void blink_and_repeat() {
  blink();
  scheduler.timer(BLINK_DELAY, blink_and_repeat);
}

// from: http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif  

  sleep(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

void alarmOn() {
#ifdef DEBUG
  Serial.print("ALARM ON: delaying ");
  Serial.print(ALARM_DELAY/1000.0);
  Serial.println("s");
#endif
  scheduler.cancel(blink_task);

  sleep(ALARM_DELAY);
  unsigned long t0 = millis();
  while(millis() - t0 < ALARM_DURATION) {
    led_on();
    buzz(1000, 1000);
    led_off();
  }
  
  blink_task = scheduler.timer(BLINK_DELAY, blink_and_repeat);
#ifdef DEBUG
  Serial.println("ALARM OFF");
#endif
}

bool check_failure_Vcc() {
  if (Vcc < VCC_ERROR) {
    digitalWrite(LED_PIN, HIGH);
    for (int i=0; i<BATTERY_ERROR_REPETITIONS; ++i) {
      buzz();
    }
    digitalWrite(LED_PIN, LOW);
    return true;
  }
  else if (Vcc < VCC_ALERT) {
    for (int i=0; i<BATTERY_ALERT_REPETITIONS; ++i) {
      blink(100, 0);
      buzz(1000, 0);
    }
  }
  return false;
}

///////////////////////////////////////////////////////////////////////////////

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZ_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // initialization message
  Serial.begin(9600);
  Serial.print("CarAlarmContinuousReading V");
  Serial.println(VERSION*0.1);
  Serial.println("Francisco Zamora-Martinez (2016)");
  digitalWrite(LED_PIN, LOW);

  Vcc = readVcc();
  
  if (check_failure_Vcc()) return;
  
  // indicates correct function by buzzing and blinking
  for (int i=0; i<BATTERY_OK_REPETITIONS; ++i) {
    blink(100, 0);
    buzz(100, 0);
  }
  
  Serial.print("START.....wait ");
  Serial.print(START_SLEEP/1000.0);
  Serial.println("s");
  sleep(START_SLEEP);

  // initialize all installed sensors
  for (int i=0; i<NUM_SENSORS; ++i) {
    sensors[i]->setup();
  }

  // schedule all required tasks
  blink_task = scheduler.timer(BLINK_DELAY, blink_and_repeat);
  alarm_task = scheduler.timer(PERIOD_SLEEP, alarm_check);
  
} // end SETUP

void alarm_check()
{
  int i, activity_detected=0;

  for (i=0; i<NUM_SENSORS; ++i) {
    if (sensors[i]->checkActivity()) {
#ifdef DEBUG
      Serial.print("Activity at sensor: ");
      Serial.println(sensors[i]->getName());
#endif
      activity_detected = 1;
    }
  }
  if (activity_detected) {
    alarmOn();
    alarm_task = scheduler.timer(REARM_DELAY, alarm_check);
  }
  else {
    alarm_task = scheduler.timer(PERIOD_SLEEP, alarm_check);
  }
}

void loop() {
  if (Vcc < VCC_ERROR) {
#ifdef DEBUG
    Serial.println("VCC_ERROR");
#endif
    sleep(100000);
  }
  else if (scheduler.pollWaiting() == ALL_IDLE) {
#ifdef DEBUG
    Serial.println("ALL_IDLE");
#endif
    sleep(100000);
  }
}
