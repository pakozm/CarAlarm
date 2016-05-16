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

   This sketch implements the logic of the receiver shield for the alarm design.
   This receiver uses an Attiny85 to filter RX 433MHz component output. A
   secure rolling code scheme is followed, as explained at:
   
   http://www.atmel.com/images/atmel-2600-avr411-secure-rolling-code-algorithm-for-wireless-link_application-note.pdf

   So, this sketch relies into a rolling counter and an AES key stored both at
   EEPROM memory of AVR chip. This sketch follows a simple algorithm which
   continuously checks RF received data, validating its CMAC and the counter
   value. If CMAC and counter are valid, CMD pin is set HIGH and the sketch
   waits ACK pin set to HIGH by the Arduino board. This procedure is repeated as
   a maximum of RFUtils::MAX_TIME_WO_RF milliseconds (normally 5 days) without
   RF interaction, going to power down mode after this time.

   A particular procedure is followed during setup which allow to program the
   AES key using the remote controller. During setup, CMD pin is set to HIGH
   during 10 seconds. Next, the sketch waits ACK pin HIGH signal from Arduino
   board. If the HIGH signal stills there after 200 milliseconds, pairing
   procedure is started. In any case, the setup procedure ends with the normal
   operation of the shield.

   Pairing procedure is acknowledged by 10 led blinks. The board waits during a
   maximum of 60 seconds for a first RF package with the RFUtils::KEY_COMMAND,
   and another maximum of 60 seconds for a second RF package which contains the
   new AES key produced by the remote controller. A led blink light of 1 second
   length indicates ok reception of the key. Four led blinks separated by 250ms
   indicate any error during pairing procedure.  
 *******************************************************************************/

/**
 *   Attiny85 PINS
 *             ____
 *   RESET   -|_|  |- 5V
 *   ON  (3) -|    |- (2) RX
 *   LED (4) -|    |- (1) CMD
 *   GND     -|____|- (0) ACK
 */

extern "C" {
#include <aes.h>
}
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/eeprom.h>
#include <CMACGenerator.h>
#include <RFUtils.h>
#include <VirtualWireCPP.h>

// #define AUTO_POWER_OFF

typedef RFUtils::message_t message_t;

const int BLOCK_SIZE = RFUtils::MESSAGE_SIZE;
const int MAC_SIZE = RFUtils::MAC_SIZE;
const int KEY_SIZE = RFUtils::KEY_SIZE;

const uint32_t MAX_COUNT_DIFF = 256;
const int EEPROM_ADDR  = 0;

const int ACK_PIN = 0;
const int CMD_PIN = 1;
const int RX_PIN  = 2;
const int ON_PIN  = 3;
const int LED_PIN = 4;

unsigned long millis_last_rf_packet = 0;

// Utility macros
#define adc_disable() (ADCSRA &= ~(1<<ADEN)) // disable ADC (before power-off)
#define adc_enable()  (ADCSRA |=  (1<<ADEN)) // re-enable ADC

VirtualWire::Receiver rx(RX_PIN);

union buf_msg_u {
  message_t msg;
  byte buffer[BLOCK_SIZE];
};
buf_msg_u buf_msg;

uint32_t count = 0;
byte key[KEY_SIZE];

unsigned long elapsedTime(unsigned long t0) {
  unsigned long t = millis();
  return t - t0;
}

bool check_count_code(uint32_t tx_count) {
  uint32_t diff = tx_count - count;
  return diff < MAX_COUNT_DIFF;
}

bool check_mac(uint32_t rx_MAC) {
  uint32_t MAC = generate_cmac(key, buf_msg.msg.count, buf_msg.msg.cmd);
  return MAC == rx_MAC;
}

void blink(unsigned long ms = 50, unsigned long post_ms = 50) {
  digitalWrite(LED_PIN, HIGH);
  delay(ms);
  digitalWrite(LED_PIN, LOW);
  delay(post_ms);
}

void commandAndACK() {
  digitalWrite(CMD_PIN, HIGH);
  while (digitalRead(ACK_PIN) == LOW);
  digitalWrite(CMD_PIN, LOW);
  blink();
}

void pair() {
  while (digitalRead(ACK_PIN) == HIGH);
  for (int i = 0; i < 10; ++i) blink();
  rx.await(60000);
  if (rx.available()) {
    int8_t nbytes = rx.recv((void*)buf_msg.buffer, BLOCK_SIZE);
    if (nbytes == BLOCK_SIZE &&
        buf_msg.msg.cmd == RFUtils::KEY_COMMAND) {
      blink();
      rx.await(60000);
      if (rx.available()) {
        int8_t nbytes = rx.recv((void*)key, BLOCK_SIZE);
        if (nbytes == BLOCK_SIZE) {
          count = 0;
          cli();
          eeprom_write_dword((uint32_t*)EEPROM_ADDR, count);
          eeprom_write_block(key, (void*)EEPROM_ADDR + sizeof(count), sizeof(key));
          eeprom_busy_wait();
          sei();
          blink();
          blink(2000);
          return;
        }

      }
    }
  }
  // error acknowledge
  blink(50,200);
  blink(50,200);
  blink(50,200);
  blink(50,200);
  // reload count and key values from EEPROM
  delay(50);
  eeprom_busy_wait();
  count = eeprom_read_dword((uint32_t*)EEPROM_ADDR);
  eeprom_read_block(key, (void*)EEPROM_ADDR + sizeof(count), sizeof(key));
}

void rf_check() {
  rx.await();
  if (rx.available()) {
    int8_t nbytes = rx.recv((void*)buf_msg.buffer, BLOCK_SIZE);
    if (nbytes == BLOCK_SIZE &&
        buf_msg.msg.cmd == RFUtils::SWITCH_COMMAND) {
      blink();
      uint32_t tx_count = buf_msg.msg.count;
      uint32_t rx_mac = buf_msg.msg.MAC;
      if (check_count_code(tx_count) && check_mac(rx_mac)) {
        blink();
        count = tx_count + 1; // keep track of the next count
        cli();
        eeprom_write_dword((uint32_t*)EEPROM_ADDR, count);
        eeprom_busy_wait();
        sei();
        commandAndACK();
        millis_last_rf_packet = millis();
      }
      delay(50);
    }
  }
}

void shutdown() {
  rx.end();
  VirtualWire::disable();
  pinMode(ACK_PIN, INPUT);
  pinMode(CMD_PIN, INPUT);
  pinMode(LED_PIN, INPUT);
  digitalWrite(ON_PIN, LOW);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();
}

void setup() {
  delay(50);
  eeprom_busy_wait();
  count = eeprom_read_dword((uint32_t*)EEPROM_ADDR);
  eeprom_read_block(key, (void*)EEPROM_ADDR + sizeof(count), sizeof(key));
  
  // put your setup code here, to run once:
  adc_disable();
  power_usi_disable();

  pinMode(ACK_PIN, INPUT);
  pinMode(CMD_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(ON_PIN, OUTPUT);

  digitalWrite(CMD_PIN, HIGH);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(ON_PIN,  HIGH);

  blink();
  delay(10000);

  digitalWrite(CMD_PIN, LOW);
  while (digitalRead(ACK_PIN) == HIGH);
  blink();
  delay(100);

  VirtualWire::begin(RFUtils::BAUD_RATE);
  rx.begin();  
  if (digitalRead(ACK_PIN) == HIGH) {
    pair();
    commandAndACK();
  }
}

void loop() {
  rf_check();
#ifdef AUTO_POWER_OFF
  if (elapsedTime(millis_last_rf_packet) > RFUtils::MAX_TIME_WO_RF) shutdown();
#endif
}
