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

#include <SensorUtils.h>

// analogic pins connection
const int POT_0_PIN = 0;
const int POT_1_PIN = 1;

long Vcc; // in mili-volts

///////////////////////////////////////////////////////////////////////////////

void setup()
{
  pinMode(13, OUTPUT);

  digitalWrite(13, LOW);
  // initialization message
  Serial.begin(9600);
  Serial.println("# Potentiometers");
  Serial.println("# Francisco Zamora-Martinez (2016)");
  // initialization delay
  delay(500);
  Vcc = SensorUtils::calibrateVcc();
} // end SETUP

void loop() {
  analogRead(POT_0_PIN); delay(50);
  long pot0 = analogRead(POT_0_PIN);
  analogRead(POT_1_PIN); delay(50);
  long pot1 = analogRead(POT_1_PIN);
  Serial.print(Vcc);
  Serial.print(" ");
  Serial.print(pot0);
  Serial.print(" ");
  Serial.println(pot1);
  delay(1000);
}
