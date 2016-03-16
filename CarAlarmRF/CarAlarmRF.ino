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
 * v0.1 2016/03/13 First version with RF.
 *******************************************************************************/
#define CAR_ALARM_INO

extern "C" {
#include <aes.h>
}
#include <JeeLib.h>
ISR(WDT_vect) { 
  Sleepy::watchdogEvent(); 
} // Setup for low power waiting
#include <AlarmUtils.h>
#include <EEPROM.h>
#include <Manchester.h>
#include <TaskTimer.h>
#include <SensorTransformations.h>
#include <CMACGenerator.h>
#include <RFUtils.h>

typedef RFUtils::message_t message_t;

const byte VERSION = 1; // firmware version divided by 10 e,g 16 = V1.6
const unsigned long ALARM_DELAY_MODE_ON = 20000; // 20 seconds
const unsigned long ALARM_DELAY_MODE_RF =   100; // 100 mili-seconds

const int MAC_LEN = 4;
const int COMMAND_LEN = sizeof(message_t);
const int KEY_LEN = 16;
const int BLOCK_SIZE = 16;
const int MAX_TASKS = 10; // maximum number of tasks for Scheduler

TaskTimerWithHeap<MAX_TASKS> scheduler;

// digital pins connection
const int RX_PIN  = 2;
const int PRG_PIN = 3;
const int EEPROM_ADDR  = 0;

bool alarm_armed = false;
uint32_t count = 0;
byte key[KEY_LEN];

union buf_msg_u {
  message_t msg;
  byte buffer[BLOCK_SIZE];
};

buf_msg_u buf_msg;

///////////////////////////////////////////////////////////////////////////////

void deepSleepMode() {
}

void awakeFromDeepSleep() {
}

bool check_count_code(uint32_t rx_count) {
  return true;
}

bool check_mac(uint32_t mac) {
  return true;
}

bool waitRX(uint32_t timeout=60000) {
  uint32_t t0 = millis();
  while(!man.receiveComplete() && millis() - t0 < timeout) delay(100);
  return millis() - t0 < timeout;
}

void pair() {
  Serial.println("PAIRING");
  for (int i=0; i<10; ++i) blink();
  blink(1000);
  buzz();
  if (waitRX() && buf_msg.msg.cmd == RFUtils::KEY_COMMAND) {
    blink();
    buzz();
    man.beginReceiveArray(BLOCK_SIZE, buf_msg.buffer);
    delay(100);
    if (waitRX()) {
      count = 0;
      EEPROM.write(EEPROM_ADDR, count);
      for (int i = 0; i < KEY_LEN; ++i) {
        EEPROM.write(i + EEPROM_ADDR + 1, key[i]);
        blink();
      }
      blink(1000);
      buzz(); buzz();
      return;
    }
  }
  // error acknowledge
  blink(); buzz();
  blink(); buzz();
  blink(); buzz();
  blink(); buzz();
}

void rf_check() {
  if (man.receiveComplete() && buf_msg.msg.cmd == RFUtils::SWITCH_COMMAND) {
    uint32_t rx_count = buf_msg.msg.count;
    uint32_t rx_mac = buf_msg.msg.MAC;
    if (check_count_code(rx_count) &&
        check_mac(rx_mac)) {
      if (alarm_armed) {
        cancelAlarm();
      deepSleepMode();
      }
      else {
        awakeFromDeepSleep();
        setupAlarm(&scheduler, ALARM_DELAY_MODE_RF);
      }
      alarm_armed = !alarm_armed;
    }
  }
  man.beginReceiveArray(BLOCK_SIZE, buf_msg.buffer);
}

///////////////////////////////////////////////////////////////////////////////

void setup()
{
  pinMode(PRG_PIN, INPUT);
  pinMode(13, OUTPUT);
  digitalWrite(PRG_PIN, HIGH);
  digitalWrite(13, LOW);
  setupAlarmPins();
  
  man.setupReceive(RX_PIN, MAN_1200);
  man.beginReceiveArray(BLOCK_SIZE, buf_msg.buffer);
  Serial.begin(9600);
  delay(1000);
  while (!Serial); // wait for Leonardo
  Serial.print("CarAlarmRF V");
  Serial.println(VERSION*0.1f);
  Serial.println("Francisco Zamora-Martinez (2016)");

  if (digitalRead(PRG_PIN) == LOW) {
    pair(); delay(1000);
    man.beginReceiveArray(BLOCK_SIZE, buf_msg.buffer);
  }
  digitalWrite(PRG_PIN, LOW);
  
  count = EEPROM.read(EEPROM_ADDR);
  Serial.print("count= "); Serial.println(count);
  for (int i=0; i<KEY_LEN; ++i) {
    key[i] = EEPROM.read(EEPROM_ADDR+i+1);
    Serial.print("KEY["); Serial.print(i); Serial.print("]= "); Serial.println(key[i]);
  }
  setupAlarm(&scheduler, ALARM_DELAY_MODE_ON);
  alarm_armed = true;
} // end SETUP

void loop() {
  if (scheduler.pollWaiting() == ALL_IDLE) {
    Serial.println("ALL_IDLE");
    sleep(100000);
  }
  rf_check();
}
