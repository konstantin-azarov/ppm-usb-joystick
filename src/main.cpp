#include "Arduino.h"
#include "EEPROM.h"
#include "Joystick.h"

const int kPpmPin = 7;
const int kDebugPin = 4;

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

Joystick_ joystick(
    /* hidReportId */ JOYSTICK_DEFAULT_REPORT_ID, 
    /* joystickType */ JOYSTICK_TYPE_MULTI_AXIS,
    /* buttonCount */ 2,
    /* hatSwitchCount */ 0,
    /* includeXAxis */ true,
    /* includeYAxis */ true,
    /* includeZAxis */ false,
    /* includeRxAxis */ false,
    /* includeRyAxis */ false,
    /* includeRzAxis */ false,
    /* includeRudder */ true,
    /* includeThrottle */ true,
    /* includeAccelerator */ false,
    /* includeBreak */ false,
    /* includeSteering */ false);

const uint8_t kCalibrationSignature = 0xBA;

uint16_t calibMin[6];
uint16_t calibMax[6];

bool calibrated = false;

bool readCalibration() {
  bool calibrated = EEPROM.read(0) == kCalibrationSignature;

  for (int i=0; i < 6; ++i) {
    if (calibrated) {
      calibMin[i] = 
        (((uint16_t)EEPROM.read(i*4 + 1)) << 8) + EEPROM.read(i * 4 + 2);
      calibMax[i] = 
        (((uint16_t)EEPROM.read(i*4 + 3)) << 8) + EEPROM.read(i * 4 + 4);
    } else {
      calibMin[i] = 0xFFFF;
      calibMax[i] = 0;
    }
  }

  if (calibrated) {
    joystick.setXAxisRange(calibMin[1], calibMax[1]);
    joystick.setYAxisRange(calibMin[2], calibMax[2]);
    joystick.setThrottleRange(calibMin[0], calibMax[0]);
    joystick.setRudderRange(calibMin[3], calibMax[3]);
    joystick.begin();
  }

  return calibrated;
}

void saveCalibration() {
  for (int i=0; i < 6; ++i) {
    EEPROM.write(i*4 + 1, (calibMin[i] >> 8) & 0xFF);
    EEPROM.write(i*4 + 2, calibMin[i] & 0xFF);
    EEPROM.write(i*4 + 3, (calibMax[i] >> 8) & 0xFF);
    EEPROM.write(i*4 + 4, calibMax[i] & 0xFF);
  }

  EEPROM.write(0, kCalibrationSignature);
}

void setup() {
  pinMode(kDebugPin, OUTPUT);

  pinMode(kPpmPin, INPUT);
  digitalWrite(kPpmPin, LOW);

  attachInterrupt(digitalPinToInterrupt(kPpmPin), pinInterrupt, CHANGE);

  Serial.begin(115200);

  calibrated = readCalibration();
}

void loop() {
  if (data_ready) {
    if (calibrated) {
      joystick.setXAxis(channels[1]);
      joystick.setYAxis(channels[2]);
      joystick.setThrottle(channels[0]);
      joystick.setRudder(channels[3]);

      joystick.setButton(
          0, channels[4] < (calibMax[4] - calibMin[4])/2 + calibMin[4]);
      joystick.setButton(
          1, channels[5] > (calibMax[5] - calibMin[5])/2 + calibMin[5]);
    } else {
      bool all_good = true;

      Serial.print(millis()); 
      for (uint8_t i=0; i < 6; ++i) {
        if (channels[i] < calibMin[i]) {
          calibMin[i] = channels[i];
        }
        if (channels[i] > calibMax[i]) {
          calibMax[i] = channels[i];
        }

        bool good = calibMax[i] > calibMin[i] + 300;

        Serial.print(" ");
        Serial.print(channels[i]);
        if (good) {
          Serial.print("+");
        }

        all_good &= good;
      }
      Serial.println();

      while (Serial.available()) {
        char c = Serial.read();
        if (all_good && c == 'w') {
          saveCalibration();
          readCalibration();
        }
      }
    }
    data_ready = false;
  }
}
