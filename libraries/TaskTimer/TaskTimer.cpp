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
#ifndef PC_TEST
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#endif
#include "TaskTimer.h"

#ifndef Arduino_h
extern void delay(unsigned long ms);
#endif

// Modified from jeelib/Ports.cpp Sleepy class implementation.
static byte backupMode = 0;
void watchdogInterrupts (char mode) {
#ifndef WDTCSR
#define WDTCSR WDTCR
#endif
  // correct for the fact that WDP3 is *not* in bit position 3!
  if (mode & bit(3))
    mode ^= bit(3) | bit(WDP3);
  // pre-calculate the WDTCSR value, can't do it inside the timed sequence
  // we only generate interrupts, no reset
  byte wdtcsr = mode >= 0 ? bit(WDIE) | mode : backupMode;
  if(mode>=0) backupMode = WDTCSR;
  MCUSR &= ~(1<<WDRF);
  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    WDTCSR |= (1<<WDCE) | (1<<WDE); // timed sequence
    WDTCSR = wdtcsr;
  }
}

/// @see http://www.nongnu.org/avr-libc/user-manual/group__avr__sleep.html
void powerDown () {
  byte adcsraSave = ADCSRA;
  ADCSRA &= ~ bit(ADEN); // disable the ADC
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    sleep_enable();
    // sleep_bod_disable(); // can't use this - not in my avr-libc version!
#ifdef BODSE
    MCUCR = MCUCR | bit(BODSE) | bit(BODS); // timed sequence
    MCUCR = (MCUCR & ~ bit(BODSE)) | bit(BODS);
#endif
  }
  sleep_cpu();
  sleep_disable();
  // re-enable what we disabled
  ADCSRA = adcsraSave;
}


static volatile byte watchdogCounter;
byte loseSomeTime(word msecs) {
  byte ok = 1;
  word msleft = msecs;
  // only slow down for periods longer than the watchdog granularity
  while (msleft >= 16) {
    char wdp = 0; // wdp 0..9 corresponds to roughly 16..8192 ms
    // calc wdp as log2(msleft/16), i.e. loop & inc while next value is ok
    for (word m = msleft; m >= 32; m >>= 1) {
      if (++wdp >= 9) break;
    }
    watchdogCounter = 0;
    watchdogInterrupts(wdp);
    powerDown();
    watchdogInterrupts(-1); // off
    // when interrupted, our best guess is that half the time has passed
    word halfms = 8 << wdp;
    msleft -= halfms;
    if (watchdogCounter == 0) {
      ok = 0; // lost some time, but got interrupted
      break;
    }
    msleft -= halfms;
  }
  // adjust the milli ticks, since we will have missed several
#if defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny85__) || defined (__AVR_ATtiny44__) || defined (__AVR_ATtiny45__)
  extern volatile unsigned long millis_timer_millis;
  millis_timer_millis += msecs - msleft;
#else
  extern volatile unsigned long timer0_millis;
  timer0_millis += msecs - msleft;
#endif
  return ok; // true if we lost approx the time planned
}

void watchdogEvent() {
  ++watchdogCounter;
}

unsigned long timeElapsedFrom(unsigned long b) {
  unsigned long a = millis();
  static const unsigned long max = 0xFFFFFFFF;
  return (a<b) ? (max-b)+a : a-b;
}

void sleep(time_type ms) {
  if (ms > 30) {
    /*
      power_adc_disable();
      delay(ms);
      power_adc_enable();
    */
    loseSomeTime(ms);
  }
  else {
    delay(ms);
  }
}
