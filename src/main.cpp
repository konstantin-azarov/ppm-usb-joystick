#include "Arduino.h"

const int kPpmPin = 3;
const int kDebugPin = 4;
const int kLedPin = 13;

uint16_t channels[6];
int8_t current_channel = -2;
bool data_ready = false;
unsigned long t0 = 0;
uint8_t l0 = 0;

void pinInterrupt() {
  uint8_t level = digitalRead(kPpmPin);
  if (level == l0) {
    return;
  }
  l0 = level;
  unsigned long t = micros();
  unsigned long dt = t - t0;

  if (level == LOW) {
    if (current_channel > -1) {
      channels[current_channel] = static_cast<uint16_t>(dt);
    } else if (dt > 10000) {
      current_channel = -1;
    }
  } else {
    if (dt < 200 || dt > 800) {
      // Malformed impulse
      current_channel = -2;
    } else if (current_channel != -2) {
      current_channel++;

      if (current_channel == 6) {
        current_channel = -2;
        data_ready = true;
      }
    }
  }

  t0 = t;
}

void setup() {
  pinMode(kLedPin, OUTPUT);
  pinMode(kDebugPin, OUTPUT);

  pinMode(kPpmPin, INPUT);
  digitalWrite(kPpmPin, LOW);

  attachInterrupt(digitalPinToInterrupt(kPpmPin), pinInterrupt, CHANGE);

  Serial.begin(115200);
}



void loop() {
  if (data_ready) {
    digitalWrite(kDebugPin, HIGH);
    Serial.print(millis()); 
    for (uint8_t i=0; i < 6; ++i) {
      Serial.print(" ");
      Serial.print(channels[i]);
    }
    Serial.println();
    data_ready = false;
    digitalWrite(kDebugPin, LOW);
  }
}
