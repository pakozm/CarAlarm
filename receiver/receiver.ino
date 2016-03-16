#include <VirtualWireCPP.h>
#include <RFUtils.h>

const int BLOCK_SIZE = RFUtils::MESSAGE_SIZE;
const int RX_PIN = 2;

typedef RFUtils::message_t message_t;
VirtualWire::Receiver rx(RX_PIN);

union buf_msg_u {
  message_t msg;
  byte buffer[BLOCK_SIZE];
};

buf_msg_u buf_msg;

void setup() {
  pinMode(13, OUTPUT);
  // put your setup code here, to run once:
  Serial.begin(9600);
  VirtualWire::begin(RFUtils::BAUD_RATE);
  rx.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  rx.await();
  if (rx.available()) {
    int8_t nbytes = rx.recv((void*)buf_msg.buffer, BLOCK_SIZE);
    Serial.print("LEN= "); Serial.println(nbytes);
    for (int i=0; i<nbytes; ++i) {
      Serial.print(buf_msg.buffer[i]); Serial.print(" ");
    }
    Serial.println();
  }
  digitalWrite(13, digitalRead(RX_PIN));
}
