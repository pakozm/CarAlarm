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

#include <JeeLib.h>
ISR(WDT_vect) { 
  Sleepy::watchdogEvent(); 
} // Setup for low power waiting
#include <AlarmUtils.h>
#include <EEPROM.h>
#include <TaskTimer.h>
#include <SensorTransformations.h>
#include <VirtualWire.h>

const byte VERSION = 08; // firmware version divided by 10 e,g 16 = V1.6
const unsigned long ALARM_DELAY_MODE_ON = 20000; // 20 seconds
const unsigned long ALARM_DELAY_MODE_RF =   100; // 100 mili-seconds

const int MAC_LEN = 4;
const int COMMAND_LEN = sizeof(message_t);
const int KEY_LEN = 16;
const int MAX_TASKS = 10; // maximum number of tasks for Scheduler

TaskTimerWithHeap<MAX_TASKS> scheduler;

// digital pins connection
const int RX_PIN  = 3;
const int PRG_PIN = 2;
const int EEPROM_POS = 0;

bool alarm_armed = false;
uint32_t seq = 0;
byte key[KEY_LEN];

struct padded_message_t {
  message_t msg;
  byte padding[VW_MAX_MESSAGE_LEN - COMMAND_LEN];
};

union {
  padded_message_t padded_msg;
  uint8_t rx_buf[VW_MAX_MESSAGE_LEN];
};
uint8_t rx_buflen = VW_MAX_MESSAGE_LEN;

///////////////////////////////////////////////////////////////////////////////

void deepSleepMode() {
}

void awakeFromDeepSleep() {
}

bool check_seq_code(uint32_t rx_seq) {
  return true;
}

bool check_mac(byte mac[MAC_LEN], message_t &msg) {
  return true;
}

void rf_check() {
  if (vw_have_message()) {
    vw_get_message(rx_buf, &rx_buflen);
    if (rx_buflen == COMMAND_LEN) {
      message_t msg = padded_msg.msg;
      byte command = msg.command;
      uint32_t rx_seq = msg.code;
      uint32_t rx_mac = msg.mac;
      if (check_seq_code(rx_seq) &&
          check_mac(rx_mac, msg)) {
        if (command == CMD_SWITCH_ALARM_STATE) {
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
    }
  }
}

///////////////////////////////////////////////////////////////////////////////

void programRF() {
  for (int i=0; i<10; ++i) blink();
  blink(1000);
  buzz();
  vw_wait_rx();
  vw_get_message(rx_buf, &rx_buflen);
  if (rx_buflen == KEY_LEN + COMMAND_LEN) {
    EEPROM.write(EEPROM_POS, 0); // code
    // write buffer key to EEPROM
    for (int i=0; i<KEY_LEN; ++i) {
      EEPROM.write(EEPROM_POS+i+1, rx_buf[i+COMMAND_LEN]);
      blink();
    }
    blink(1000);
    buzz();
  }
}

void setup()
{
  pinMode(PRG_PIN, INPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  Serial.begin(9600);
  delay(100);
  while (!Serial); // wait for Leonardo
  print("CarAlarmRF V");
  println(VERSION*0.1f);
  println("Francisco Zamora-Martinez (2016)");
  vw_set_rx_pin(RX_PIN);
  vw_setup(2000);
  vw_rx_start();
  delay(100);
  if (digitalRead(PRG_PIN) == HIGH) programRF();
  seq = EEPROM.read(EEPROM_POS);
  for (int i=0; i<KEY_LEN; ++i) {
    key[i] = EEPROM.read(EEPROM_POS+i+1);
  }
  setupAlarm(&scheduler, ALARM_DELAY_MODE_ON);
  alarm_armed = true;
} // end SETUP

void loop() {
  if (scheduler.pollWaiting() == ALL_IDLE) {
    println("ALL_IDLE");
    sleep(100000);
  }
  rf_check();
}
