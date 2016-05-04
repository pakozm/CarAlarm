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

   The alarm can be armed/disarmed using a RF remote control. In this case, the
   delay lengths are different. PCINT0 interrupt is used to detect activity in
   RF. A specific shield with an ATtiny85 IC continuously listens for activity
   in 433MHz RF band. When validated activity is detected (remote control
   message is received) this shields activates HIGH a pin in PCINT0 group of
   the Arduino Atmega328.

   The alarm disconnects after RFUtils::MAX_TIME_WO_RF milliseconds, normally
   set to five days.

   The normal operation of the alarm drains approx. 17 mA of power. When the
   siren is working this power consumption increases up to 80 mA. In sleep
   (after five days without interaction) the alarm drains 7 mA of power.

   Setup procedure: depending on alarm battery level the starting procedure
   changes, indicating if everything is ok to work, if the battery needs to be
   replaced/charged, or if battery is so low that the system won't run:

   1. Correct function is advertised by BATTERY_OK_REPETITIONS short buzzes.

   2. Low battery is advertised by BATTERY_ALERT_REPETITIONS long buzzes, but the
   alarm stills running.

   3. Very low battery is advertised by BATTERY_ERROR_REPETITIONS short buzzes
   and the system won't run.

   Change Log:
   v0.3 2016/04/18 Added PCINT0 interrupt handler for RX shield comm.
   v0.2 2016/03/27 Updated to improve power consumption.
   v0.1 2016/03/13 First version with RF.
 *******************************************************************************/

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
#include <RFUtils.h>

ISR(WDT_vect) {
  Sleepy::watchdogEvent();
}

const byte VERSION = 3; // firmware version divided by 10 e,g 16 = V1.6

const unsigned long START_SLEEP_MODE_ON = 60000; // 60 seconds
const unsigned long ALARM_DELAY_MODE_ON = 20000; // 20 seconds

const unsigned long START_SLEEP_MODE_RF =  1000; // in mili-seconds
const unsigned long ALARM_DELAY_MODE_RF =  1000; // in mili-seconds

const unsigned long ALL_IDLE_SLEEP_LENGTH = 60000; // in mili-seconds

TaskTimerWithHeap<20> scheduler;

// digital pins connection
const int PRG_PIN      = 3;
const int RX_ACK_PIN   = 7;
const int RX_CMD_PIN   = 8;

bool alarm_armed = false;
volatile bool do_rf_check = false;
unsigned long millis_last_rf_packet = 0;

///////////////////////////////////////////////////////////////////////////////

unsigned long elapsedTime(unsigned long t0) {
  unsigned long t = millis();
  return t - t0;
}

void lowPowerMode() {
    ADCSRA &= ~(1 << ADEN);
    power_adc_disable();
}

void normalPowerMode() {
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
  while (digitalRead(RX_CMD_PIN) == LOW);
  sendACK();
  buzz();
}

void rf_check() {
  if (Serial) Serial.println("RF CHECK");
  sendACK();
  if (alarm_armed) { // disarm
    cancelAlarm();
    cancelAlarmPins();
    scheduler.clear();
    lowPowerMode();
  }
  else { // arm
    normalPowerMode();
    setupAlarmPins();
    setupAlarm(&scheduler, ALARM_DELAY_MODE_RF, START_SLEEP_MODE_RF);
  }
  alarm_armed = !alarm_armed;
  if (Serial) {
    Serial.print("ARMED: "); Serial.println(alarm_armed);
  }
  millis_last_rf_packet = millis();
}

void configure_power()
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

void shutdown() {
  if (Serial) {
    Serial.println("SHUTING DOWN");
  }
  blink(1000, 0);
  pinMode(PRG_PIN, INPUT_PULLUP);
  pinMode(RX_ACK_PIN, INPUT_PULLUP);
  pinMode(RX_CMD_PIN, INPUT_PULLUP);
  // pinMode(13, INPUT_PULLUP);
  cancelAlarmPins();

  unsigned char clockSource = 0;
  if (TCCR2B & CS22) clockSource |= (1 << CS22);
  if (TCCR2B & CS21) clockSource |= (1 << CS21);
  if (TCCR2B & CS20) clockSource |= (1 << CS20);
  // Remove the clock source to shutdown Timer2
  TCCR2B &= ~(1 << CS22);
  TCCR2B &= ~(1 << CS21);
  TCCR2B &= ~(1 << CS20);
  power_timer2_disable();

  ADCSRA &= ~(1 << ADEN);
  power_adc_disable();

  power_timer1_disable();

  power_timer0_disable();

  power_spi_disable();

  power_usart0_disable();

  power_twi_disable();

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();
  sleep_enable();
  sleep_bod_disable();
  // sei();
  sleep_cpu();
  // we never returns from here
}

ISR(PCINT0_vect) {
  if (Serial) Serial.println("PCINT0");
  if (digitalRead(RX_CMD_PIN) == HIGH) do_rf_check = true;
}

void initialiseInterrupt(){
  cli();		// switch interrupts off while messing with their settings  
  PCICR |= 0b00000001;  // Enable Port B interrupts
  PCMSK0 |= 0b00000001; // Enable PB0, which is PCINT0, physical pin 14, digital pin 8 (RX_CMD_PIN)
  sei();		// turn interrupts back on
}

///////////////////////////////////////////////////////////////////////////////

void setup()
{
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  pinMode(PRG_PIN, INPUT_PULLUP);
  pinMode(RX_ACK_PIN, OUTPUT);
  pinMode(RX_CMD_PIN, INPUT);
  pinMode(13, OUTPUT);

  digitalWrite(13, LOW);
  // pinMode(13, INPUT_PULLUP);

  digitalWrite(RX_ACK_PIN, HIGH);

  setupAlarmPins();

  Serial.begin(9600);
  delay(500);
  while (!Serial); // wait for Leonardo
  Serial.print("# CarAlarmRF V");
  Serial.println(VERSION * 0.1f);
  Serial.println("# Francisco Zamora-Martinez (2016)");

  while (digitalRead(RX_CMD_PIN) == HIGH);
  digitalWrite(RX_ACK_PIN, LOW);

  if (digitalRead(PRG_PIN) == LOW) pair();

  setupAlarm(&scheduler, ALARM_DELAY_MODE_ON, START_SLEEP_MODE_ON);
  alarm_armed = true;

  configure_power();
  initialiseInterrupt();
} // end SETUP

void loop() {
  if (Serial) Serial.println("LOOP");
  id_type id = scheduler.pollWaiting();
  if (do_rf_check) {
    rf_check();
    do_rf_check = false;  
  }
  else if (id == ALL_IDLE) {
    if (Serial) Serial.println("ALL_IDLE");
    sleep(ALL_IDLE_SLEEP_LENGTH);
  }
  if (elapsedTime(millis_last_rf_packet) > RFUtils::MAX_TIME_WO_RF) {
    scheduler.clear();
    shutdown();
  }
}

