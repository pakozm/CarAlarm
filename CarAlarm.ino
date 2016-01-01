/*
 * This file is part of CarAlarm an Arduino sketch for a car alarm system.

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

/*
  Change Log:
  v0.2 2016/01/02 Added readVcc() routine.
  v0.1 2016/01/01 First draft: buzzer, PIR and accelerometer.
*/

/*
  TODO: add interrupt-based controller for PIR sensor
*/

const byte VERSION = 01; // firmware version divided by 10 e,g 16 = V1.6
const bool DEBUG = true;
const unsigned long PERIOD_SLEEP = 10; // 10 ms
const int DEBUG_ARMING_SLEEP = 1000; // 1 s
const int RELEASE_ARMING_SLEEP = 60000; // 60 s
const unsigned long BLINK_DELAY = 10000; // 10 s
const unsigned long RELEASE_REARM_DELAY = 60000; // 60 s
const unsigned long DEBUG_REARM_DELAY = 4000; // 4 s
const int NUM_SENSORS = 2;

// digital pins connection
const int PIR_PIN = 2;
const int LED_PIN = 13;
const int BUZ_PIN = 12;

// analogic pins connection
const int ACC_X_PIN = 0;
const int ACC_Y_PIN = 1;
const int ACC_Z_PIN = 2;

// accelerometer threshold
const float ACC_TH = 10.0f;

long Vcc; // in mili-volts

///////////////////////////////////////////////////////////////////////////

class AlarmSensor {
public:
  virtual ~AlarmSensor() {}
  // initializes the sensor (probably calibrating its rest values)
  virtual void setup()=0;
  // checks if sensor is active or not
  virtual bool checkActivity()=0;
  //
  virtual const char * const getName()=0;
};

class PIRSensor : public AlarmSensor {
public:
  PIRSensor(int pin) : pin(pin) { };
  virtual void setup() {
    pinMode(pin, INPUT);
  }
  virtual bool checkActivity() {
    int val = digitalRead(pin);
    return (val == HIGH);
  }
  virtual const char * const getName() { return "PIR"; }
private:
  int pin, count;
};

class AccelerometerSensor : public AlarmSensor {
  public:
  AccelerometerSensor(int x_pin, int y_pin, int z_pin, float threshold) :
    x_pin(x_pin), y_pin(y_pin), z_pin(z_pin),
    threshold2(threshold*threshold) { };
  virtual void setup() {
    x_ref = analogRead(x_pin);
    y_ref = analogRead(y_pin);
    z_ref = analogRead(z_pin);
  }
  virtual bool checkActivity() {
    int x_val, y_val, z_val;
    x_val = analogRead(x_pin);
    y_val = analogRead(y_pin);
    z_val = analogRead(z_pin);
    float dx=x_val-x_ref, dy=y_val-y_ref, dz=z_val-z_ref;
    return (dx*dx + dy*dy + dz*dy) > threshold2;
  }
  virtual const char * const getName() { return "ACC"; }
private:
  int x_pin, y_pin, z_pin;
  int x_ref, y_ref, z_ref;
  float threshold2;
};

///////////////////////////////////////////////////////////////////////////

PIRSensor pir_sensor(PIR_PIN);
AccelerometerSensor acc_sensor(ACC_X_PIN, ACC_Y_PIN, ACC_Z_PIN, ACC_TH);

AlarmSensor *sensors[NUM_SENSORS] = { &pir_sensor, &acc_sensor };

void blink(unsigned long ms=100, unsigned long post_ms=200) {
  digitalWrite(LED_PIN, HIGH); delay(ms);
  digitalWrite(LED_PIN, LOW); delay(post_ms);
}

void buzz(unsigned long ms=100, unsigned long post_ms=200) {
  digitalWrite(BUZ_PIN, HIGH); delay(ms);
  digitalWrite(BUZ_PIN, LOW); delay(post_ms);
}

// from: http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

void alarmOn() {
  if (DEBUG) Serial.println("ALARM ON");
  digitalWrite(LED_PIN, HIGH);
  delay(4000);
  digitalWrite(LED_PIN, LOW);
  if (DEBUG) Serial.println("ALARM OFF");
}

unsigned long last_time = 0;
unsigned long alarm_delay = 0;

bool alarm_delayed() {
  if (alarm_delay > 0) {
    unsigned long time_delta = millis() - last_time;
    if (time_delta < alarm_delay) alarm_delay -= time_delta;
    else alarm_delay = 0;
  }
  last_time = millis();
  return (alarm_delay > 0);
}

///////////////////////////////////////////////////////////////////////////////

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZ_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // initialization message
  Serial.begin(9600);
  Serial.print("Car Alarm V");
  Serial.println(VERSION*0.1);
  Serial.println("Francisco Zamora-Martinez (2016)");
  digitalWrite(LED_PIN, LOW);

  Vcc = readVcc();

  if (Vcc < VCC_THRESHOLD) {
    digitalWrite(LED_PIN, HIGH);
    for (int i=0; i<10; ++i) {
      buzz(100, 0);
    }
    digitalWrite(LED_PIN, LOW);
    return;
  }
  
  // indicates correct function by buzzing 2 times
  buzz();
  buzz();
    
  // indicates correct function by blinking 10 times
  for (int i=0; i<10; i++) blink();

  Serial.println("ARMING.....wait 60s");
  if (DEBUG) delay(DEBUG_ARMING_SLEEP);
  else delay(RELEASE_ARMING_SLEEP);

  // initialize all installed sensors
  for (int i=0; i<NUM_SENSORS; ++i) sensors[i]->setup();
} // end SETUP

void loop()
{
  if (Vcc < VCC_THRESHOLD) {
    buzz();
    delay(10000);
    break;
  }
  else {
    int i, activity_detected=0;

    if (!alarm_delayed()) {
      for (i=0; i<NUM_SENSORS; ++i) {
        if (sensors[i]->checkActivity()) {
          if (DEBUG) {
            Serial.print("Activity at sensor: ");
            Serial.println(sensors[i]->getName());
          }
          activity_detected = 1;
        }
      }
      if (activity_detected) {
        alarmOn();
        if (!DEBUG) alarm_delay = RELEASE_REARM_DELAY;
        else alarm_delay = DEBUG_REARM_DELAY;
      }
    }
  
    if (millis() % BLINK_DELAY == 0) blink(100, 0);
    delay(PERIOD_SLEEP);
  }
}
