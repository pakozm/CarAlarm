/*
   This file is part of CarAlarm an Arduino project for a car alarm system.

   Copyright (c) 2016 Francisco Zamora-Martinez (pakozm@gmail.com)

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
   IN THE SOFTWARE.
*/

/*******************************************************************************
   Summary:

   This alarm uses different sensors to detect activity inside the car. When any
   of them sense activity, the alarm will start making noise for a particular
   duration.

   The alarm has a START_DELAY time which allow the user to arm the alarm and
   leave the car. Once this delay passes, the alarm executes a routine every
   PERIOD_SLEEP looking for activity in the sensors. When activity is sensed, a
   delay of ALARM_DELAY is used to allow manual disconnection of the alarm. After
   this time the alarm starts making noise during ALARM_DURATION time. After this
   time the alarm waits REARM_DELAY before rearming alarm again.


   Setup procedure: depending on alarm battery level the starting procedure
   changes, indicating if everything is ok to work, if the battery needs to be
   replaced/charged, or if battery is so low that the system won't run:

   1. Correct function is advertised by BATTERY_OK_REPETITIONS short buzzes.

   2. Low battery is advertised by BATTERY_ALERT_REPETITIONS long buzzes, but the
   alarm stills running.

   3. Very low battery is advertised by BATTERY_ERROR_REPETITIONS short buzzes
   and the system won't run.


   Change Log:
   v0.1 2016/03/13 First version with RF.
 *******************************************************************************/
#define CAR_ALARM_INO

extern "C" {
#include <aes.h>
}
#include <AlarmUtils.h>
#include <EEPROM.h>
#include <avr/eeprom.h>
#include <TaskTimer.h>
#include <SensorTransformations.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <JeeLib.h>

ISR(WDT_vect) { Sleepy::watchdogEvent(); }

const byte VERSION = 1; // firmware version divided by 10 e,g 16 = V1.6

const unsigned long START_SLEEP_MODE_ON = 60000; // 60 seconds
const unsigned long ALARM_DELAY_MODE_ON = 20000; // 20 seconds

const unsigned long START_SLEEP_MODE_RF =  5000; // in mili-seconds
const unsigned long ALARM_DELAY_MODE_RF =  5000; // in mili-seconds

const int MAX_TASKS = 15; // maximum number of tasks for Scheduler

const int RF_CHECK_PERIOD =  2000; // in mili-seconds

TaskTimerWithHeap<MAX_TASKS> scheduler;

// digital pins connection
const int PRG_PIN      = 3;
const int RX_ACK_PIN   = 7;
const int RX_CMD_PIN   = 8;

id_type rf_check_task;
bool alarm_armed = false;

///////////////////////////////////////////////////////////////////////////////

void deepSleepMode() {
  ADCSRA &= ~(1 << ADEN);
  power_adc_disable();
  
}

void awakeFromDeepSleep() {
  power_adc_enable();
  ADCSRA |= (1 << ADEN);
}

void sendACK() {
  digitalWrite(RX_ACK_PIN, HIGH);
  delay(100);
  digitalWrite(RX_ACK_PIN, LOW);
}

void pair() {
  digitalWrite(RX_ACK_PIN, HIGH);
  Serial.println("PAIRING");
  for (int i = 0; i < 10; ++i) blink();
  blink(1000);
  buzz();
  digitalWrite(RX_ACK_PIN, LOW);
  while(digitalRead(RX_CMD_PIN) == LOW);
  sendACK();
  buzz();
}

void rf_check(void *) {
  if (Serial) Serial.println("RF CHECK");
  if (digitalRead(RX_CMD_PIN) == HIGH) {
    sendACK();
    if (alarm_armed) {
      cancelAlarm();
      cancelAlarmPins();
      deepSleepMode();
    }
    else {
      awakeFromDeepSleep();
      setupAlarmPins();
      setupAlarm(&scheduler, ALARM_DELAY_MODE_RF, START_SLEEP_MODE_RF);
    }
    alarm_armed = !alarm_armed;
  }
  rf_check_task = scheduler.timer(RF_CHECK_PERIOD, rf_check);
}

void low_power_mode()
{
  // Temporary clock source variable 
  unsigned char clockSource = 0;

  // Disable the analog comparator by setting the ACD bit
  // (bit 7) of the ACSR register to one.
  ACSR = B10000000;

  if (TCCR2B & CS22) clockSource |= (1 << CS22);
  if (TCCR2B & CS21) clockSource |= (1 << CS21);
  if (TCCR2B & CS20) clockSource |= (1 << CS20);
  
  // Remove the clock source to shutdown Timer2
  TCCR2B &= ~(1 << CS22);
  TCCR2B &= ~(1 << CS21);
  TCCR2B &= ~(1 << CS20);
  
  power_timer2_disable();

  power_spi_disable();
  if (!Serial) power_usart0_disable();
  power_twi_disable();
}

///////////////////////////////////////////////////////////////////////////////

void setup()
{
  pinMode(PRG_PIN, INPUT);
  pinMode(RX_ACK_PIN, OUTPUT);
  pinMode(RX_CMD_PIN, INPUT);
  pinMode(13, OUTPUT);
  
  digitalWrite(PRG_PIN, HIGH);
  digitalWrite(13, LOW);
  pinMode(13, INPUT);

  digitalWrite(RX_ACK_PIN, HIGH);
  
  setupAlarmPins();

  Serial.begin(9600);
  delay(500);
  while (!Serial); // wait for Leonardo
  Serial.print("CarAlarmRF V");
  Serial.println(VERSION * 0.1f);
  Serial.println("Francisco Zamora-Martinez (2016)");

  while(digitalRead(RX_CMD_PIN) == HIGH);
  digitalWrite(RX_ACK_PIN, LOW);

  if (digitalRead(PRG_PIN) == LOW) {
    pair();
  }
  digitalWrite(PRG_PIN, LOW);

  setupAlarm(&scheduler, ALARM_DELAY_MODE_ON, START_SLEEP_MODE_ON);
  alarm_armed = true;

  low_power_mode();
  rf_check_task = scheduler.timer(RF_CHECK_PERIOD, rf_check);
} // end SETUP

void loop() {
  if (scheduler.pollWaiting() == ALL_IDLE) {
    sleep(60000);
  }
}
