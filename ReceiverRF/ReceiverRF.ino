/**
   Attiny85 PINS
            ____
   RESET  -|    |- 5V
   A3 (3) -|    |- (2) A1
   A2 (4) -|    |- (1) PWM
   GND    -|____|- (0) PWM

*/

extern "C" {
#include <aes.h>
}
#include <avr/power.h>
#include <EEPROM.h>
#include <avr/eeprom.h>
#include <CMACGenerator.h>
#include <RFUtils.h>
#include <VirtualWireCPP.h>

typedef RFUtils::message_t message_t;

const int BLOCK_SIZE = RFUtils::MESSAGE_SIZE;
const int MAC_SIZE = RFUtils::MAC_SIZE;
const int KEY_SIZE = RFUtils::KEY_SIZE;

const uint32_t COUNT_MAX = 0xFFFFFFFF;
const uint32_t MAX_COUNT_DIFF = 256;
const int EEPROM_ADDR  = 0;

const int ACK_PIN = 0;
const int CMD_PIN = 1;
const int RX_PIN  = 2;
const int LED_PIN = 4;

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

bool check_count_code(uint32_t tx_count) {
  uint32_t diff;
  if (tx_count < count) {
    diff = (COUNT_MAX - count) + tx_count;
  }
  else {
    diff = tx_count - count;
  }
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
        int8_t nbytes = rx.recv((void*)buf_msg.buffer, BLOCK_SIZE);
        if (nbytes == BLOCK_SIZE) {
          count = 0;
          eeprom_write_dword((uint32_t*)EEPROM_ADDR, count);
          for (int i = 0; i < KEY_SIZE; ++i) {
            EEPROM.write(i + EEPROM_ADDR + sizeof(count), buf_msg.buffer[i]);
            blink();
          }
          blink(1000);
          return;
        }
      }
    }
  }
  // error acknowledge
  blink();
  blink();
  blink();
  blink();
}

void rf_check() {
  rx.await();
  if (rx.available()) {
    int8_t nbytes = rx.recv((void*)buf_msg.buffer, BLOCK_SIZE);
    if (nbytes == BLOCK_SIZE &&
        buf_msg.msg.cmd == RFUtils::SWITCH_COMMAND) {
      uint32_t tx_count = buf_msg.msg.count;
      uint32_t rx_mac = buf_msg.msg.MAC;
      if (check_count_code(tx_count) &&
          check_mac(rx_mac)) {
        blink();
        count = tx_count + 1; // keep track of the next count
        eeprom_write_dword((uint32_t*)EEPROM_ADDR, count);
        commandAndACK();
      }
      delay(500);
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  adc_disable();
  power_usi_disable();

  pinMode(ACK_PIN, INPUT);
  pinMode(CMD_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(CMD_PIN, HIGH);
  digitalWrite(LED_PIN, LOW);

  blink();
  delay(10000);

  VirtualWire::begin(RFUtils::BAUD_RATE);
  rx.begin();

  digitalWrite(CMD_PIN, LOW);
  while (digitalRead(ACK_PIN) == HIGH);
  blink();
  delay(200);
  if (digitalRead(ACK_PIN) == HIGH) {
    pair();
    commandAndACK();
  }

  count = eeprom_read_dword((uint32_t*)EEPROM_ADDR);
  for (int i = 0; i < KEY_SIZE; ++i) {
    key[i] = EEPROM.read(i + EEPROM_ADDR + sizeof(count));
  }
}

void loop() {
  rf_check();
}

