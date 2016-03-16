/**
 * @file VirtualWire.cpp
 * @version 1.0
 *
 * @section License
 * Copyright (C) 2008-2013, Mike McCauley (Author/VirtualWire),
 * Mikael Patel (C++ refactoring/editing).
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General
 * Public License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA  02111-1307  USA
 *
 * @section Description
 * VirtualWire is a library that provides features to send short
 * messages, without addressing, retransmit or acknowledgment, a bit
 * like UDP over wireless, using ASK (Amplitude Shift
 * Keying). Supports a number of inexpensive radio transmitters and
 * receivers. 
 */

#include "VirtualWireCPP.h"
#include <util/crc16.h>

#define membersof(x) (sizeof(x)/sizeof(x[0]))

const uint8_t 
VirtualWire::symbols[] PROGMEM = {
  0xd,  0xe,  0x13, 0x15, 0x16, 0x19, 0x1a, 0x1c, 
  0x23, 0x25, 0x26, 0x29, 0x2a, 0x2c, 0x32, 0x34
};

uint8_t VirtualWire::s_mode = 0;

uint16_t 
VirtualWire::CRC(uint8_t* ptr, uint8_t count)
{
  uint16_t crc = 0xffff;
  while (count-- > 0) 
    crc = _crc_ccitt_update(crc, *ptr++);
  return (crc);
}

uint8_t 
VirtualWire::symbol_6to4(uint8_t symbol)
{
  for (uint8_t i = 0; i < membersof(symbols); i++)
    if (symbol == pgm_read_byte(&symbols[i]))
      return (i);
  return (0);
}

/** Current transmitter/receiver for interrupt handler access */
static VirtualWire::Transmitter* transmitter = 0;
static VirtualWire::Receiver* receiver = 0;

#if defined(__AVR_ATtiny25__)		\
 || defined(__AVR_ATtiny45__)		\
 || defined(__AVR_ATtiny85__)

/** Prescale table for 8-bit Timer1. Index is prescale setting */
static const uint16_t prescale[] PROGMEM = {
  0, 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384 
};

#else

/** Prescale table for 16-bit Timer1. Index is prescale setting */
static const uint16_t prescale[] PROGMEM = {
  0, 1, 8, 64, 256, 1024
};

#endif

/**
 * Calculate timer setting, prescale and count value, given speed (bps),
 * number of bits in timer. Returns zero(0) if fails otherwise prescale
 * value/index, and timer top.
 * @param[in] speed bits per second, transmitter/receiver.
 * @param[in] bits number of bits in counter (8/16O).
 * @param[out] nticks timer top value.
 * @return prescale or zero(0).
 */
static uint8_t
timer_setting(uint16_t speed, uint8_t bits, uint16_t* nticks) 
{
  uint16_t max_ticks = (1 << bits) - 1;
  uint8_t res = 0;
  uint8_t i;
  for (i = membersof(prescale) - 1; i > 0; i--) {
    uint16_t scale = (uint16_t) pgm_read_word(&prescale[i]);
    uint16_t count = (F_CPU / scale) / speed;
    if (count > res && count < max_ticks) {
      *nticks = count;
      res = i;
    }
  }
  return (res);
}

void 
VirtualWire::Receiver::PLL()
{
  // Integrate each sample
  if (m_sample) m_integrator++;

  if (m_sample != m_last_sample) {
    // Transition, advance if ramp > TRANSITION otherwise retard
    m_pll_ramp += 
      ((m_pll_ramp < RAMP_TRANSITION) ? RAMP_INC_RETARD : RAMP_INC_ADVANCE);
    m_last_sample = m_sample;
  }
  else {
    // No transition: Advance ramp by standard INC (== MAX/BITS samples)
    m_pll_ramp += RAMP_INC;
  }
  if (m_pll_ramp >= RAMP_MAX) {
    // Add this to the 12th bit of vw_rx_bits, LSB first. The last 12
    // bits are kept 
    m_bits >>= 1;

    // Check the integrator to see how many samples in this cycle were
    // high. If < 5 out of 8, then its declared a 0 bit, else a 1;
    if (m_integrator >= INTEGRATOR_THRESHOLD)
      m_bits |= 0x800;

    m_pll_ramp -= RAMP_MAX;

    // Clear the integral for the next cycle
    m_integrator = 0; 

    if (m_active) {
      // We have the start symbol and now we are collecting message
      // bits, 6 per symbol, each which has to be decoded to 4 bits
      if (++m_bit_count >= (BITS_PER_SYMBOL * 2)) {
	// Have 12 bits of encoded message == 1 byte encoded. Decode
	// as 2 lots of 6 bits into 2 lots of 4 bits. The 6 lsbits are
	// the high nybble.
	uint8_t data = 
	  (symbol_6to4(m_bits & SYMBOL_MASK) << 4) | 
	  symbol_6to4(m_bits >> BITS_PER_SYMBOL);
	
	// The first decoded byte is the byte count of the following
	// message the count includes the byte count and the 2
	// trailing FCS bytes. REVISIT: may also include the ACK flag
	// at 0x40.
	if (m_length == 0) {
	  // The first byte is the byte count. Check it for
	  // sensibility. It cant be less than 4, since it includes
	  // the bytes count itself and the 2 byte FCS 
	  m_count = data;
	  if (m_count < MESSAGE_MIN || m_count > MESSAGE_MAX) {
	    // Stupid message length, drop the whole thing
	    m_active = false;
	    m_bad++;
	    return;
	  }
	}
	m_buffer[m_length++] = data;
	if (m_length >= m_count) {
	  // Got all the bytes now
	  m_active = false;
	  m_good++;
	  // Better come get it before the next one starts
	  m_done = true;
	}
	m_bit_count = 0;
      }
    }

    // Not in a message, see if we have a start symbol
    else if (m_bits == START_SYMBOL) {
      // Have start symbol, start collecting message
      m_active = true;
      m_bit_count = 0;
      m_length = 0;
      // Too bad if you missed the last message
      m_done = false;
    }
  }
}

bool 
VirtualWire::begin(uint16_t speed, uint8_t mode)
{
  // Number of prescaled ticks needed
  uint16_t nticks = 0;

  // Bit values for CS0[2:0]
  uint8_t prescaler;

  // Set sleep mode
  s_mode = mode;

#if defined(__AVR_ATtiny25__)		\
 || defined(__AVR_ATtiny45__)		\
 || defined(__AVR_ATtiny85__)
  // Figure out prescaler value and counter match value
  prescaler = timer_setting(speed * SAMPLES_PER_BIT, 8, &nticks);
  if (!prescaler) return (0);

  // Turn on CTC mode / Output Compare pins disconnected
  TCCR1 = _BV(PWM1A) | prescaler;

  // Number of ticks to count before firing interrupt
  OCR1A = uint8_t(nticks);
#else
  // Figure out prescaler value and counter match value
  prescaler = timer_setting(speed * SAMPLES_PER_BIT, 16, &nticks);
  if (!prescaler) return (0);

  // Output Compare pins disconnected, and turn on CTC mode
  TCCR1A = 0; 
  TCCR1B = _BV(WGM12);

  // Convert prescaler index to TCCRnB prescaler bits CS10, CS11, CS12
  TCCR1B |= prescaler;

  // Caution: special procedures for setting 16 bit regs
  // is handled by the compiler
  OCR1A = nticks;
#endif
  enable();
  return (1);
}

void
VirtualWire::enable()
{
#if defined(__AVR_ATtiny25__)		\
 || defined(__AVR_ATtiny45__)		\
 || defined(__AVR_ATtiny85__)
  // Enable interrupt on compare
  TIMSK |= _BV(OCIE1A);
#else
  // Enable interrupt
#ifdef TIMSK1
  TIMSK1 |= _BV(OCIE1A);
#else
  TIMSK |= _BV(OCIE1A);
#endif
#endif
}

void
VirtualWire::disable()
{
#if defined(__AVR_ATtiny25__)		\
 || defined(__AVR_ATtiny45__)		\
 || defined(__AVR_ATtiny85__)
  // Enable interrupt on compare
  TIMSK &= ~_BV(OCIE1A);
#else
  // Enable interrupt
#ifdef TIMSK1
  TIMSK1 &= ~_BV(OCIE1A);
#else
  TIMSK &= ~_BV(OCIE1A);
#endif
#endif
}

VirtualWire::Receiver::Receiver(uint8_t rx) : 
  m_pin(rx)
{
  receiver = this;
}

bool 
VirtualWire::Receiver::await(unsigned long ms)
{
  // Allow low power mode while waiting
  unsigned long start = millis();
  while (!m_done && (ms == 0 || ((millis() - start) < ms))) {
  }
  return (m_done);
}

int8_t
VirtualWire::Receiver::recv(void* buf, uint8_t len, uint32_t ms)
{
  // Message available?
  if (!m_done && (ms == 0 || !await(ms))) return (0);

  // Message check-sum error
  if (CRC(m_buffer, m_length) != CHECK_SUM) return (-1);
    
  // Wait until done is set before reading length then remove
  // bytecount and FCS  
  uint8_t rxlen = m_length - 3;
    
  // Copy message (good or bad). Skip count byte
  if (len > rxlen) len = rxlen;
  memcpy(buf, m_buffer + 1, len);
    
  // OK, got that message thanks
  m_done = false;
    
  // Return actual number of bytes received
  return (len);
}

const uint8_t 
VirtualWire::Transmitter::header[HEADER_MAX] PROGMEM = {
  0x2a, 0x2a, 0x2a, 0x2a, 0x2a, 0x2a, 0x38, 0x2c
};

VirtualWire::Transmitter::Transmitter(uint8_t tx) :
  m_pin(tx)
{
  transmitter = this;
  memcpy_P(m_buffer, header, sizeof(header));
}

bool 
VirtualWire::Transmitter::begin()
{
  m_index = 0;
  m_bit = 0;
  m_sample = 0;
  m_enabled = true;
  return (1);
}

void 
VirtualWire::Transmitter::await()
{
  while (m_enabled);
}

bool 
VirtualWire::Transmitter::send(void* buf, uint8_t len)
{
  // Check that the message is not too large
  if (len > PAYLOAD_MAX) return (0);

  uint8_t *tp = transmitter->m_buffer + HEADER_MAX;
  uint8_t *bp = (uint8_t*) buf;
  uint16_t crc = 0xffff;
  
  // Wait for transmitter to become available
  await();

  // Encode the message length
  uint8_t count = len + 3;
  crc = _crc_ccitt_update(crc, count);
  *tp++ = pgm_read_byte(&symbols[count >> 4]);
  *tp++ = pgm_read_byte(&symbols[count & 0xf]);

  // Encode the message into 6 bit symbols. Each byte is converted into 
  // 2 X 6-bit symbols, high nybble first, low nybble second
  for (uint8_t i = 0; i < len; i++) {
    crc = _crc_ccitt_update(crc, bp[i]);
    *tp++ = pgm_read_byte(&symbols[bp[i] >> 4]);
    *tp++ = pgm_read_byte(&symbols[bp[i] & 0xf]);
  }

  // Append the FCS, 16 bits before encoding (4 X 6-bit symbols after
  // encoding) Caution: VirtualWire expects the _ones_complement_ of the CCITT
  // CRC-16 as the FCS VirtualWire sends FCS as low byte then hi byte
  crc = ~crc;
  *tp++ = pgm_read_byte(&symbols[(crc >> 4)  & 0xf]);
  *tp++ = pgm_read_byte(&symbols[crc & 0xf]);
  *tp++ = pgm_read_byte(&symbols[(crc >> 12) & 0xf]);
  *tp++ = pgm_read_byte(&symbols[(crc >> 8)  & 0xf]);

  // Total number of 6-bit symbols to send
  m_length = HEADER_MAX + (count * 2);

  // Start the low level interrupt handler sending symbols
  return (begin());
}

/**
 * This is the interrupt service routine called when timer1
 * overflows. Its job is to output the next bit from the transmitter
 * (every 8 calls) and to call the PLL code if the receiver is enabled.
 */
ISR(TIMER1_COMPA_vect)
{
  // Check if the receiver pin should be sampled
  if ((receiver != 0 && receiver->m_enabled)
      && (transmitter == 0 || !transmitter->m_enabled)) {
    receiver->m_sample = digitalRead(receiver->m_pin);
  }
    
  // Do transmitter stuff first to reduce transmitter bit jitter due 
  // to variable receiver processing
  if (transmitter != 0) {
    if (transmitter->m_enabled && transmitter->m_sample++ == 0) {
      // Send next bit. Symbols are sent LSB first. Finished sending the
      // whole message? (after waiting one bit period since the last bit)
      if (transmitter->m_index >= transmitter->m_length) {
	transmitter->end();
	transmitter->m_msg_count++;
      }
      else {
	digitalWrite(transmitter->m_pin, 
		     transmitter->m_buffer[transmitter->m_index] & 
		     (1 << transmitter->m_bit++));
	if (transmitter->m_bit >= VirtualWire::BITS_PER_SYMBOL) {
	  transmitter->m_bit = 0;
	  transmitter->m_index++;
	}
      }
    }
    if (transmitter->m_sample >= VirtualWire::SAMPLES_PER_BIT)
      transmitter->m_sample = 0;
  }
  if ((receiver != 0 && receiver->m_enabled)
      && (transmitter == 0 || !transmitter->m_enabled))
    receiver->PLL();
}

