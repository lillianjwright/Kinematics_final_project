/*
   6-DOF Robot Arm — Bluetooth byte control (HC-05, Arduino Mega)
   - Jog joints with bytes: 16..27
   - Save pose: 12
   - Run: 14   Pause: 15   Reset/Stop: 13
   - Speed: 102..249  -> delay = value/10 ms (clamped)
*/

#include <Servo.h>

// ------------------- Hardware -------------------
Servo servo01, servo02, servo03, servo04, servo05, servo06;
#define LED_PIN 14

// ------------------- Globals --------------------
int servo1PPos, servo2PPos, servo3PPos, servo4PPos, servo5PPos, servo6PPos; // current positions
int speedDelay = 20;            // ms between degree steps
int dataIn = -1;                 // last byte from BT
int m = 0;                       // mode / last command

// ----- Pose sequence (saved in the order you press SAVE) -----
struct Pose { int s1, s2, s3, s4, s5, s6; };
Pose seq[50];
int poseCount = 0;

// ------------------- Forward Declarations -------------------
void savePose();
void printPoses();
inline void clampSpeed();
bool pollControl();                  // check for PAUSE/RUN/RESET/speed during motion
bool moveToPose(const Pose& p);      // move all joints together to target
void playSequence();                 // 0 -> 1 -> ... -> last, repeat until RESET


// ============================================================
//                          SETUP
// ============================================================
void setup() {
  pinMode(LED_PIN, OUTPUT);

  servo01.attach(5);
  servo02.attach(6);
  servo03.attach(7);
  servo04.attach(8);
  servo05.attach(9);
  servo06.attach(10);

  Serial.begin(9600);     // USB
  Serial1.begin(9600);    // HC-05 default
  Serial.println("Ready");

  // Initial pose
  servo1PPos = 0;   servo01.write(servo1PPos);
  servo2PPos = 70;  servo02.write(servo2PPos);
  servo3PPos = 15;  servo03.write(servo3PPos);
  servo4PPos = 105;   servo04.write(servo4PPos);
  servo5PPos = 140;   servo05.write(servo5PPos);
  servo6PPos = 160;  servo06.write(servo6PPos);
}

// ============================================================
//                           LOOP
// ============================================================
void loop() {
  // --- Read raw command bytes from Bluetooth (App Inventor Send1ByteNumber) ---
  while (Serial1.available()) {
    dataIn = Serial1.read();            // 0..255
    Serial.print("RX: "); Serial.println(dataIn);

    // Speed 102..249 => 10..24 ms
    if (dataIn >= 102 && dataIn <= 249) {
      speedDelay = dataIn / 10;
      clampSpeed();
    }

    // Mode / command
    if (dataIn >= 0 && dataIn <= 27) {
      m = dataIn;
    }
  }

  // ----------------- Jogging (holds until a new byte changes m) -----------------
  // Joint 1
  while (m == 16) { if (Serial1.available()) { int v = Serial1.read(); if (v >= 0) m = v; }
    if (servo1PPos < 180) servo1PPos++; servo01.write(servo1PPos); delay(speedDelay); }
  while (m == 17) { if (Serial1.available()) { int v = Serial1.read(); if (v >= 0) m = v; }
    if (servo1PPos > 0)   servo1PPos--; servo01.write(servo1PPos); delay(speedDelay); }

  // Joint 2
  while (m == 19) { if (Serial1.available()) { int v = Serial1.read(); if (v >= 0) m = v; }
    if (servo2PPos < 180) servo2PPos++; servo02.write(servo2PPos); delay(speedDelay); }
  while (m == 18) { if (Serial1.available()) { int v = Serial1.read(); if (v >= 0) m = v; }
    if (servo2PPos > 0)   servo2PPos--; servo02.write(servo2PPos); delay(speedDelay); }

  // Joint 3
  while (m == 20) { if (Serial1.available()) { int v = Serial1.read(); if (v >= 0) m = v; }
    if (servo3PPos < 180) servo3PPos++; servo03.write(servo3PPos); delay(speedDelay); }
  while (m == 21) { if (Serial1.available()) { int v = Serial1.read(); if (v >= 0) m = v; }
    if (servo3PPos > 0)   servo3PPos--; servo03.write(servo3PPos); delay(speedDelay); }

  // Joint 4
  while (m == 23) { if (Serial1.available()) { int v = Serial1.read(); if (v >= 0) m = v; }
    if (servo4PPos < 180) servo4PPos++; servo04.write(servo4PPos); delay(speedDelay); }
  while (m == 22) { if (Serial1.available()) { int v = Serial1.read(); if (v >= 0) m = v; }
    if (servo4PPos > 0)   servo4PPos--; servo04.write(servo4PPos); delay(speedDelay); }

  // Joint 5
  while (m == 25) { if (Serial1.available()) { int v = Serial1.read(); if (v >= 0) m = v; }
    if (servo5PPos < 180) servo5PPos++; servo05.write(servo5PPos); delay(speedDelay); }
  while (m == 24) { if (Serial1.available()) { int v = Serial1.read(); if (v >= 0) m = v; }
    if (servo5PPos > 0)   servo5PPos--; servo05.write(servo5PPos); delay(speedDelay); }

  // Joint 6
  while (m == 26) { if (Serial1.available()) { int v = Serial1.read(); if (v >= 0) m = v; }
    if (servo6PPos < 180) servo6PPos++; servo06.write(servo6PPos); delay(speedDelay); }
  while (m == 27) { if (Serial1.available()) { int v = Serial1.read(); if (v >= 0) m = v; }
    if (servo6PPos > 0)   servo6PPos--; servo06.write(servo6PPos); delay(speedDelay); }

  // ----------------- SAVE pose -----------------
  if (m == 12) {
    savePose();
    m = 0;
  }

  // ----------------- RUN sequence (strict saved order) -----------------
  if (m == 14) {
    if (poseCount < 2) {
      Serial.println("RUN ignored: save at least 2 poses.");
      m = 0;
    } else {
      printPoses();
      playSequence();          // repeat until RESET (13)
    }
  }
}

// ============================================================
//                     Helper Implementations
// ============================================================
void savePose() {
  if (poseCount >= 50) return;
  seq[poseCount++] = { servo1PPos, servo2PPos, servo3PPos, servo4PPos, servo5PPos, servo6PPos };
  Serial.print("Saved pose #"); Serial.println(poseCount - 1);
}

void printPoses() {
  Serial.println("---- Pose list ----");
  for (int i = 0; i < poseCount; i++) {
    Serial.print(i); Serial.print(": ");
    Serial.print(seq[i].s1); Serial.print(",");
    Serial.print(seq[i].s2); Serial.print(",");
    Serial.print(seq[i].s3); Serial.print(",");
    Serial.print(seq[i].s4); Serial.print(",");
    Serial.print(seq[i].s5); Serial.print(",");
    Serial.println(seq[i].s6);
  }
}

inline void clampSpeed() {
  if (speedDelay < 5)  speedDelay = 5;
  if (speedDelay > 50) speedDelay = 50;
}

// Poll for control bytes during motion.
// Returns false if RESET (13) requested; keeps PAUSE/RUN and speed live.
bool pollControl() {
  if (Serial1.available()) {
    int b = Serial1.read();
    if (b >= 102 && b <= 249) { speedDelay = b / 10; clampSpeed(); }
    else if (b == 13) return false;     // RESET
    else if (b == 15) {                 // PAUSE until RUN
      int x = 0;
      while (x != 14) {
        if (Serial1.available()) x = Serial1.read();
        delay(1);
      }
    } else {
      // ignore other mode bytes during RUN
    }
  }
  return true;
}

// Move all joints together 1°/step until they match target pose.
// Returns false if RESET requested during move.
bool moveToPose(const Pose& p) {
  bool reached = false;
  while (!reached) {
    if (!pollControl()) return false;   // RESET aborts
    reached = true;

    if (servo1PPos != p.s1) { servo1PPos += (p.s1 > servo1PPos) ? 1 : -1; servo01.write(servo1PPos); reached = false; }
    if (servo2PPos != p.s2) { servo2PPos += (p.s2 > servo2PPos) ? 1 : -1; servo02.write(servo2PPos); reached = false; }
    if (servo3PPos != p.s3) { servo3PPos += (p.s3 > servo3PPos) ? 1 : -1; servo03.write(servo3PPos); reached = false; }
    if (servo4PPos != p.s4) { servo4PPos += (p.s4 > servo4PPos) ? 1 : -1; servo04.write(servo4PPos); reached = false; }
    if (servo5PPos != p.s5) { servo5PPos += (p.s5 > servo5PPos) ? 1 : -1; servo05.write(servo5PPos); reached = false; }
    if (servo6PPos != p.s6) { servo6PPos += (p.s6 > servo6PPos) ? 1 : -1; servo06.write(servo6PPos); reached = false; }

    delay(speedDelay);
  }
  return true;   // reached target
}

// Play 0 -> 1 -> ... -> poseCount-1, repeat while m==14; stop on RESET (13).
void playSequence() {
  clampSpeed();
  while (m == 14) {
    for (int i = 0; i < poseCount; i++) {
      if (!moveToPose(seq[i])) {   // RESET
        m = 0;
        return;
      }
    }
  }
}
