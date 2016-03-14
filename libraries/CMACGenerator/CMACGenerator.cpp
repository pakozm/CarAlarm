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
extern "C" {
#include <aes.h>
}
#include "Arduino.h"

const int BLOCK_SIZE = 16;
const int PADDING_SIZE = 11;

struct premessage_t {
  uint32_t count;
  byte cmd;
  byte padding[11]; // 11 padding bytes
};

void block_left_shift(byte *data) {
  for (int i=0; i<BLOCK_SIZE-1; ++i) {
    data[i] = data[i] << 1;
    data[i] = (data[i] & 0xFE) | ((data[i+1] & 0x80) >> 7);
  }
  data[BLOCK_SIZE-1] = data[BLOCK_SIZE-1] << 1;
}

void block_xor(byte *data, byte *other) {
  for (int i=0; i<BLOCK_SIZE; ++i) {
    data[i] = data[i] ^ other[i];
  }
}

byte msb(byte *data) {
  return data[0] & 0x80;
}

/**
 * @note: http://csrc.nist.gov/publications/nistpubs/800-38B/SP_800-38B.pdf
 *
 * @note: http://www.atmel.com/images/atmel-2600-avr411-secure-rolling-code-algorithm-for-wireless-link_application-note.pdf
 */
uint32_t generate_cmac(byte *key, uint32_t count, byte cmd) {
  union {
    premessage_t premsg;
    byte data[BLOCK_SIZE];
    uint32_t MAC[BLOCK_SIZE/4];
  };
  byte R128[BLOCK_SIZE];
  byte subkey[BLOCK_SIZE];
  premsg.count = count;
  premsg.cmd = cmd;
  for (int i=0; i<PADDING_SIZE; ++i) premsg.padding[i] = 0;
  premsg.padding[0] = 0x80;
  // R128 constant declaration
  for (int i=0; i<BLOCK_SIZE; ++i) R128[i] = 0;
  R128[0] = 0x80;
  R128[BLOCK_SIZE-1] = 0x87;
  // subkey generation
  for (int i=0; i<BLOCK_SIZE; ++i) subkey[i] = 0;
  aesCipher(key, subkey);
  block_left_shift(subkey);
  if (msb(subkey)) block_xor(subkey, R128);
  block_left_shift(subkey);
  if (msb(subkey)) block_xor(subkey, R128);
  // apply xor with subkey
  block_xor(data, subkey);
  aesCipher(key, data);
  return MAC[0];
}
