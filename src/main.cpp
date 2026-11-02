#include <Arduino.h>
#include <AccelStepper.h>

const int pinStep  = 6;
const int pinDir   = 5;
const int pinStep2 = 12;
const int pinDir2  = 11;

const int pinRx = A0;
const int pinTx = A1;

AccelStepper stepperX(AccelStepper::DRIVER, pinStep, pinDir);    // alpha (F/B)
AccelStepper stepperY(AccelStepper::DRIVER, pinStep2, pinDir2);  // beta (R/L)

// ---- Tuning ----
const float DEADZONE_DEG = 3.0;     // ignore tilt smaller than this
const float MAX_TILT_DEG = 30.0;    // tilt at which you reach full speed
const float MAX_SPEED    = 700.0;   // steps/sec at full tilt
const long  X_MIN = -2000, X_MAX = 2000;   // travel limits in steps, from startup position
const long  Y_MIN = -2000, Y_MAX = 2000;   // CHANGE to match your board
const unsigned long TIMEOUT_MS = 200;      // stop if no data this long
// Flip to -1 if an axis moves the wrong way
const int DIR_X = 1;
const int DIR_Y = 1;

String boardbuffer = "";
float tiltX = 0, tiltY = 0;
unsigned long lastData = 0;

float tiltToSpeed(float angleDeg) {
  float mag = fabs(angleDeg);
  if (mag < DEADZONE_DEG) return 0;
  float scaled = (mag - DEADZONE_DEG) / (MAX_TILT_DEG - DEADZONE_DEG);
  scaled = constrain(scaled, 0.0f, 1.0f);
  return (angleDeg > 0 ? 1 : -1) * scaled * MAX_SPEED;
}

void applySpeed(AccelStepper &s, float speed, long minPos, long maxPos) {
  long pos = s.currentPosition();
  if ((speed > 0 && pos >= maxPos) || (speed < 0 && pos <= minPos)) speed = 0;
  s.setSpeed(speed);
}

void setup() {
  Serial.begin(115200);
  Serial1.setPins(pinRx, pinTx);
  Serial1.begin(9600);

  stepperX.setMaxSpeed(MAX_SPEED);
  stepperY.setMaxSpeed(MAX_SPEED);
  stepperX.setCurrentPosition(0);  // assumes piece starts at center
  stepperY.setCurrentPosition(0);
  lastData = millis();
}

void loop() {
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\r') continue;

    if (c == '\n') {
      boardbuffer.trim();
      int commaPos = boardbuffer.indexOf(',');
      if (commaPos > 0) {
        // sender transmits angle/100, so *100 gives degrees
        tiltX = boardbuffer.substring(0, commaPos).toFloat() * 100.0;
        tiltY = boardbuffer.substring(commaPos + 1).toFloat() * 100.0;
        lastData = millis();

          Serial.print("RX alpha: ");
          Serial.print(tiltX, 2);
          Serial.print("  beta: ");
          Serial.println(tiltY, 2);
      }
      boardbuffer = "";
    } else {
      boardbuffer += c;
    }
  }

  float vx = tiltToSpeed(tiltX) * DIR_X;
  float vy = tiltToSpeed(tiltY) * DIR_Y;

  if (millis() - lastData > TIMEOUT_MS) { vx = 0; vy = 0; }

  applySpeed(stepperX, vx, X_MIN, X_MAX);
  applySpeed(stepperY, vy, Y_MIN, Y_MAX);

  stepperX.runSpeed();
  stepperY.runSpeed();
}