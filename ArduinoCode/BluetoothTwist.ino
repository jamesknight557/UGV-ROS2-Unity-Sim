#include <SoftwareSerial.h>

// HC-05 serial
SoftwareSerial bt(10, 9); // RX, TX (Arduino side)

// Motor shield pins from your sample (M1 and M3 used for tracks)
const int E_LEFT  = 3;   // PWM for M1
const int M_LEFT  = 4;   // DIR for M1

const int E_RIGHT = 5;   // PWM for M3
const int M_RIGHT = 8;   // DIR for M3

// Safety timeout: stop if no command received
unsigned long lastCmdMs = 0;
const unsigned long CMD_TIMEOUT_MS = 500;

// Clamp helper
int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void setMotor(int pwmPin, int dirPin, int pwm) {
  pwm = clampInt(pwm, -255, 255);
  if (pwm >= 0) {
    digitalWrite(dirPin, LOW);        // forward (matches your sample for M1/M3)
    analogWrite(pwmPin, pwm);
  } else {
    digitalWrite(dirPin, HIGH);       // reverse
    analogWrite(pwmPin, -pwm);
  }
}

void stopTracks() {
  analogWrite(E_LEFT, 0);
  analogWrite(E_RIGHT, 0);
}

void setup() {
  pinMode(E_LEFT, OUTPUT);
  pinMode(M_LEFT, OUTPUT);
  pinMode(E_RIGHT, OUTPUT);
  pinMode(M_RIGHT, OUTPUT);

  stopTracks();

  // USB serial for debugging
  Serial.begin(9600);

  // HC-05 default is commonly 9600 in data mode (AT mode differs)
  bt.begin(9600);

  lastCmdMs = millis();
  Serial.println("Ready: expecting 'D,<L>,<R>\\n' over Bluetooth");
}

bool parseDriveCommand(const String& line, int &l, int &r) {
  // Expected: D,<left>,<right>
  if (line.length() < 3) return false;
  if (line.charAt(0) != 'D') return false;
  if (line.charAt(1) != ',') return false;

  int firstComma = line.indexOf(',', 2);
  if (firstComma < 0) return false;

  String ls = line.substring(2, firstComma);
  String rs = line.substring(firstComma + 1);

  l = ls.toInt();
  r = rs.toInt();
  return true;
}

void loop() {
  // Read a line from Bluetooth
  if (bt.available()) {
    String line = bt.readStringUntil('\n');
    line.trim();
    Serial.print("BT RX: ");
    Serial.println(line);

    int l = 0, r = 0;
    if (parseDriveCommand(line, l, r)) {
      l = clampInt(l, -255, 255);
      r = clampInt(r, -255, 255);

      setMotor(E_LEFT,  M_LEFT,  l);
      setMotor(E_RIGHT, M_RIGHT, r);

      lastCmdMs = millis();

      // Optional debug
      // Serial.print("CMD L="); Serial.print(l);
      // Serial.print(" R="); Serial.println(r)

    }
  }

  // Safety stop if commands stop arriving
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    stopTracks();
  }
}
