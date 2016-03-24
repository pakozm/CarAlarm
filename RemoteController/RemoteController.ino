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
extern "C" {
#include <aes.h>
}
#include <avr/sleep.h>
#include <EEPROM.h>
#include <avr/eeprom.h>
#include <CMACGenerator.h>
#include <RFUtils.h>
#include <VirtualWireCPP.h>

typedef RFUtils::message_t message_t;

// Utility macros
#define adc_disable() (ADCSRA &= ~(1<<ADEN)) // disable ADC (before power-off)
#define adc_enable()  (ADCSRA |=  (1<<ADEN)) // re-enable ADC

const byte VERSION = 1; // firmware version divided by 10 e,g 16 = V1.6

const int TX_REPLAY_TIME  = 3000;
const int TX_REPLAY_DELAY =    9;

const int KEY_SIZE = RFUtils::KEY_SIZE; // in bytes
const uint8_t NUM_RANDOM_BITS = 2;
const uint8_t RANDOM_BITS_MASK = (1 << NUM_RANDOM_BITS) - 1;
const uint8_t NUM_PUSHES_PER_BYTE = 8 / NUM_RANDOM_BITS;
const int EEPROM_ADDR = 0;

const int PAIR_PIN = 2; // dip pin 7
const int LED_PIN  = 4; // dip pin 3
const int TX_PIN   = 3; // dip pin 2

VirtualWire::Transmitter tx(TX_PIN);
uint32_t count;
uint8_t key[KEY_SIZE];
bool pairing_mode = false; // true when read(PAIR_PIN)==HIGH

// messages and payloads are always 16 bytes length
void send_command(byte cmd, byte *payload = 0, byte payload_length = 0) {
  uint32_t MAC = generate_cmac(key, count, cmd);
  message_t msg;
  for (int i=0; i<RFUtils::PADDING_SIZE; ++i) msg.padding[i] = 0xAA;
  msg.count = count; 
  msg.cmd = cmd;
  msg.MAC = MAC;
  tx.send((void*)&msg, sizeof(message_t));
  tx.await();
  if (payload_length != 0) {
    delay(100);
    tx.send((void*)payload, payload_length);
    tx.await();
  }
}

void led_on() {
  digitalWrite(LED_PIN, HIGH);
}

void led_off() {
  digitalWrite(LED_PIN, LOW);
}

void blink(unsigned long ms = 50, unsigned long post_ms = 200) {
  led_on();
  delay(ms);
  led_off();
  delay(post_ms);
}

bool active_pair_button() {
  return digitalRead(PAIR_PIN) == HIGH;
}

uint8_t get_random_uint8() {
  uint8_t b = 0;
  while (active_pair_button());
  for (int j = 0; j < NUM_PUSHES_PER_BYTE; ++j) {
    long t0 = millis();
    while (!active_pair_button());
    while (active_pair_button());
    long t1 = millis();
    uint8_t d = (t1 - t0) & RANDOM_BITS_MASK;
    b = (b << NUM_RANDOM_BITS) | d;
  }
  return b;
}

void pair() {
  delay(2000); // wait two seconds before starting pairing
  for (int i=0; i<4; ++i) blink();
  for (int i = 0; i < KEY_SIZE; ++i) {
    key[i] = get_random_uint8();
    blink(10);
  }
  send_command(RFUtils::KEY_COMMAND, key, KEY_SIZE);
  count = 0;
  eeprom_write_dword((uint32_t*)EEPROM_ADDR, count);
  for (int i = 0; i < KEY_SIZE; ++i) EEPROM.write(i + EEPROM_ADDR + sizeof(count), key[i]);
  blink(2000);
}

//////////////////////////////////////////////////////////////////////////

void shutdown() {
  tx.end();
  VirtualWire::disable();
  pinMode(TX_PIN, INPUT);
  pinMode(LED_PIN, INPUT);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();
}

void setup() {
  adc_disable();
  pinMode(TX_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(PAIR_PIN, INPUT);
  digitalWrite(LED_PIN, LOW);

  VirtualWire::begin(RFUtils::BAUD_RATE);
  tx.begin();
  
  if (active_pair_button()) pairing_mode = true;

  count = eeprom_read_dword((uint32_t*)EEPROM_ADDR);
  for (int i = 0; i < KEY_SIZE; ++i) {
    key[i] = EEPROM.read(i + EEPROM_ADDR + sizeof(count));
  }
}

void loop() {
  if (pairing_mode) {
    pair();
  }
  else {
    blink(50,50);
    unsigned long t0 = millis();
    while(millis() - t0 < TX_REPLAY_TIME) {
      send_command(RFUtils::SWITCH_COMMAND);
      delay(TX_REPLAY_DELAY);
    }
    ++count;
    eeprom_write_dword((uint32_t*)EEPROM_ADDR, count);
    blink(50,50);
  }
  shutdown();
}
