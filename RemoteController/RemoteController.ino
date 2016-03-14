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
// #define DEBUG
extern "C" {
#include <aes.h>
}
#include <avr/sleep.h>
#include <EEPROM.h>
#include <Manchester.h>
#include <CMACGenerator.h>

// Utility macros
#define adc_disable() (ADCSRA &= ~(1<<ADEN)) // disable ADC (before power-off)
#define adc_enable()  (ADCSRA |=  (1<<ADEN)) // re-enable ADC

const byte VERSION = 01; // firmware version divided by 10 e,g 16 = V1.6
const long REF_CAL = 1120000L;

const int TX_SPEED = 2000; // speed of data transfer bps
const int KEY_LENGTH = 16; // in bytes
const uint8_t NUM_RANDOM_BITS = 4;
const uint8_t RANDOM_BITS_MASK = (1 << NUM_RANDOM_BITS) - 1;
const uint8_t NUM_PUSHES_PER_BYTE = 8/NUM_RANDOM_BITS;
const int EEPROM_ADDR = 0;

const int PAIR_PIN    = 2; // dip pin 7
const int LED_PIN     = 4; // dip pin 3
const int TX_PIN      = 3; // dip pin 2

const byte SWITCH_COMMAND = 0x01;
const byte KEY_COMMAND    = 0x20;

uint32_t count;
uint8_t key[KEY_LENGTH];
bool pairing_mode = false; // true when read(PAIR_PIN)==HIGH

struct message_t {
  uint32_t count;
  byte cmd;
  uint32_t MAC;
  message_t(uint32_t count, byte cmd, uint32_t MAC) :
  count(count), cmd(cmd), MAC(MAC) {
  }
};

void send_command(byte cmd, byte *payload=0, byte payload_length=0) {
  uint32_t MAC = generate_cmac(key, count, cmd);
  message_t msg(count, cmd, MAC);
#ifdef DEBUG
  Serial.print(msg.count);
  Serial.print(" ");
  Serial.print(msg.cmd);
  Serial.print(" ");
  Serial.println(msg.MAC);
#endif
  man.transmitArray(sizeof(message_t), (uint8_t*)&msg);
  if (payload_length != 0) {
    man.transmitArray(payload_length, payload);
  }
  ++count;
  EEPROM.write(EEPROM_ADDR, count);
}

void led_on() {
  digitalWrite(LED_PIN, HIGH);
}

void led_off() {
  digitalWrite(LED_PIN, LOW);
}

void blink(unsigned long ms=100, unsigned long post_ms=200) {
  led_on(); 
  delay(ms);
  led_off(); 
  delay(post_ms);
}

bool active_pair_button() {
  return digitalRead(PAIR_PIN)==HIGH;
}

uint8_t get_random_uint8() {
  uint8_t b = 0;
  while(active_pair_button());
  for (int j=0; j<NUM_PUSHES_PER_BYTE; ++j) {
    long t0 = millis();
    while(!active_pair_button());
    while(active_pair_button());
    long t1 = millis();
    uint8_t d = (t1 - t0) & RANDOM_BITS_MASK;
    b = (b<<NUM_RANDOM_BITS) | d;
  }
#ifdef DEBUG  
  Serial.print(b);
  Serial.print(" ");
#endif
  return b;
}

void pair() {
#ifdef DEBUG
  Serial.println("Starting pairing");
  Serial.print("MASK "); 
  Serial.println(RANDOM_BITS_MASK);
  Serial.print("NUM PUSHES "); 
  Serial.println(NUM_PUSHES_PER_BYTE);
#endif
  delay(2000); // wait two seconds before starting pairing
  blink();
  blink();
#ifdef DEBUG
  Serial.print("KEY: ");
#endif
  for (int i=0; i<KEY_LENGTH; ++i) {
    key[i] = get_random_uint8();
    blink(10);
  }
#ifdef DEBUG
  Serial.println("");
#endif
  send_command(KEY_COMMAND, key, KEY_LENGTH);
#ifdef DEBUG
  Serial.println("Pairing done");
#endif
  count = 0;
  EEPROM.write(EEPROM_ADDR, count);
  for (int i=0; i<KEY_LENGTH; ++i) EEPROM.write(i+EEPROM_ADDR+1, key[i]);
  blink(2000);
}

void show_error() {
  blink(1000);
  blink(1000);
}

//////////////////////////////////////////////////////////////////////////

void shutdown() {
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

#ifdef DEBUG
  Serial.begin(9600);
  delay(100);
  while (!Serial); // wait for Leonardo
  Serial.print("RemoteController V");
  Serial.println(VERSION*0.1f);
  Serial.println("Francisco Zamora-Martinez (2016)");
#endif
  
  man.workAround1MhzTinyCore();
  man.setupTransmit(TX_PIN, MAN_1200);
  
  if (active_pair_button()) pairing_mode = true;
  count = EEPROM.read(EEPROM_ADDR);

  for (int i=0; i<KEY_LENGTH; ++i) {
    key[i] = EEPROM.read(i+EEPROM_ADDR+1);
  }
  
#ifdef DEBUG
  Serial.print("COUNT: "); 
  Serial.println(count);
  Serial.print("STORED KEY: ");
  for (int i=0; i<KEY_LENGTH; ++i) {
    Serial.print(key[i]); 
    Serial.print(" ");
  }
  Serial.println();
#endif
}

void loop() {
  if (pairing_mode) {
    pair();
  }
  else {
    send_command(SWITCH_COMMAND);
    blink();
  }
  shutdown();
}
