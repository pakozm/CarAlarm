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
/*#include <JeeLib.h>
ISR(WDT_vect) { 
  Sleepy::watchdogEvent(); 
} // Setup for low power waiting
*/
#include <AlarmUtils.h>
#include <EEPROM.h>
#include <VirtualWireCPP.h>
#include <TaskTimer.h>
#include <SensorTransformations.h>
#include <CMACGenerator.h>
#include <RFUtils.h>
#include <LowPower.h>

typedef RFUtils::message_t message_t;

const byte VERSION = 1; // firmware version divided by 10 e,g 16 = V1.6
const unsigned long ALARM_DELAY_MODE_ON = 20000; // 20 seconds
const unsigned long ALARM_DELAY_MODE_RF =   100; // 100 mili-seconds

const int BLOCK_SIZE = RFUtils::MESSAGE_SIZE;
const int MAC_SIZE = RFUtils::MAC_SIZE;
const int KEY_SIZE = RFUtils::KEY_SIZE;
const int MAX_TASKS = 10; // maximum number of tasks for Scheduler
const int COUNT_MAX = 0xFFFFFFFF;
const int MAX_COUNT_DIFF = 256;

TaskTimerWithHeap<MAX_TASKS> scheduler;

// digital pins connection
const int RX_PIN  = 2;
const int PRG_PIN = 3;
const int EEPROM_ADDR  = 0;

bool alarm_armed = false;
uint32_t count = 0;
byte key[KEY_SIZE];

VirtualWire::Receiver rx(RX_PIN);

union buf_msg_u {
  message_t msg;
  byte buffer[BLOCK_SIZE];
};

buf_msg_u buf_msg;

///////////////////////////////////////////////////////////////////////////////

void deepSleepMode() {
  rx.await();
}

void awakeFromDeepSleep() {
}

bool check_count_code(uint32_t rx_count) {
  uint32_t diff;
  if (rx_count < count) {
    diff = COUNT_MAX - count + rx_count;
  }
  else {
    diff = rx_count - count;
  }
  return diff < MAX_COUNT_DIFF;
}

bool check_mac(uint32_t rx_MAC) {
  uint32_t MAC = generate_cmac(key, buf_msg.msg.count, buf_msg.msg.cmd);
  return MAC == rx_MAC;
}

void pair() {
  Serial.println("PAIRING");
  for (int i=0; i<10; ++i) blink();
  blink(1000);
  buzz();
  rx.await(60000);
  if (rx.available()) {
    int8_t nbytes = rx.recv((void*)buf_msg.buffer, BLOCK_SIZE);
    if (nbytes == BLOCK_SIZE &&
        buf_msg.msg.cmd == RFUtils::KEY_COMMAND) {
      blink();
      buzz();
      rx.await(60000);
      if (rx.available()) {
        int8_t nbytes = rx.recv((void*)buf_msg.buffer, BLOCK_SIZE);
        if (nbytes == BLOCK_SIZE) {
          count = 0;
          EEPROM.write(EEPROM_ADDR, count);
          for (int i = 0; i < KEY_SIZE; ++i) {
            EEPROM.write(i + EEPROM_ADDR + 1, buf_msg.buffer[i]);
            blink();
          }
          blink(1000);
          buzz(); buzz();
          return;
        }
      }
    }
  }
  // error acknowledge
  blink(); buzz();
  blink(); buzz();
  blink(); buzz();
  blink(); buzz();
}

void rf_check() {
  if (rx.available()) {
    int8_t nbytes = rx.recv((void*)buf_msg.buffer, BLOCK_SIZE);

    for (int i=0; i<nbytes; ++i) {
      Serial.print(buf_msg.buffer[i]); Serial.print(" ");
    }
    Serial.println();

    if (nbytes != BLOCK_SIZE) return;
    if (buf_msg.msg.cmd != RFUtils::SWITCH_COMMAND) return;
    
    uint32_t rx_count = buf_msg.msg.count;
    uint32_t rx_mac = buf_msg.msg.MAC;
    if (check_count_code(rx_count) &&
        check_mac(rx_mac)) {
      Serial.print("ARMED= "); Serial.println(!alarm_armed);
      count = rx_count+1; // keep track of the next count
      if (alarm_armed) {
        //cancelAlarm();
        deepSleepMode();
      }
      else {
        awakeFromDeepSleep();
        //setupAlarm(&scheduler, ALARM_DELAY_MODE_RF);
      }
      alarm_armed = !alarm_armed;
    }
  }
}

///////////////////////////////////////////////////////////////////////////////

void setup()
{
  pinMode(PRG_PIN, INPUT);
  pinMode(13, OUTPUT);
  digitalWrite(PRG_PIN, HIGH);
  digitalWrite(13, LOW);
  setupAlarmPins();

  VirtualWire::begin(RFUtils::BAUD_RATE);
  rx.begin();

  Serial.begin(9600);
  delay(1000);
  while (!Serial); // wait for Leonardo
  Serial.print("CarAlarmRF V");
  Serial.println(VERSION*0.1f);
  Serial.println("Francisco Zamora-Martinez (2016)");

  if (digitalRead(PRG_PIN) == LOW) {
    pair(); delay(1000);
  }
  digitalWrite(PRG_PIN, LOW);
  
  count = EEPROM.read(EEPROM_ADDR);
  Serial.print("count= "); Serial.println(count);
  for (int i=0; i<KEY_SIZE; ++i) {
    key[i] = EEPROM.read(EEPROM_ADDR+i+1);
    Serial.print("KEY["); Serial.print(i); Serial.print("]= "); Serial.println(key[i]);
  }
  //setupAlarm(&scheduler, ALARM_DELAY_MODE_ON);
  alarm_armed = true;
} // end SETUP

void loop() {
  if (scheduler.pollWaiting() == ALL_IDLE) {
    Serial.println("ALL_IDLE");
    rx.await();
  }
  rf_check();
}
