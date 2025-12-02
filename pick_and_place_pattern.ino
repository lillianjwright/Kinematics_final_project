#include <Servo.h>
#include <math.h>

// =========================
// Servo configuration
// =========================
const int BASE_PIN        = 5;
const int SHOULDER_PIN    = 6;
const int ELBOW_PIN       = 7;
const int WRIST_PITCH_PIN = 9;
const int WRIST_ROT_PIN   = 8;
const int GRIPPER_PIN     = 10;

Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo wristPitchServo;
Servo wristRotServo;
Servo gripperServo;

// =========================
// Robot geometry (mm)
// =========================
const double L1 = 125.0;    // base -> elbow
const double L2 = 130.0;    // elbow -> wrist/claw (simplified)
const double CLAW_LENGTH = 130.0;  // wrist joint -> tip along tool axis

// =========================
// Servo angle mapping
// =========================

// Direction (+1 or -1) and offsets (deg) for each joint
const int BASE_DIR        = -1;
const int SHOULDER_DIR    = 1;
const int ELBOW_DIR       = -1;
const int WRIST_PITCH_DIR = 1;
const int WRIST_ROT_DIR   = -1;

const double BASE_OFFSET        = 165.0;  // servo angle when theta1 = 0 rad
const double SHOULDER_OFFSET    = -20.0;   // when theta2 = 0
const double ELBOW_OFFSET       = 15.0;   // when theta3 = 0
const double WRIST_PITCH_OFFSET = 90.0;   // when theta4 = 0
const double WRIST_ROT_OFFSET   = 105.0;  // when theta5 = 0

// Gripper angles
const int GRIPPER_OPEN_ANGLE  = 160;   // adjust
const int GRIPPER_CLOSE_ANGLE = 90;    // adjust

// =========================
// Positions (mm) in BASE frame
// =========================

// Single duck reference position
double duckX = 100.0;
double duckY = 0.0;
double duckZ = -120.0;

// Box (drop) position
double boxX  = -150.0;
double boxY  = 0.0;
double boxZ  = 0.0;

// Lift height above duck/box
const double LIFT_Z_OFFSET = 40.0;  // mm

// =========================
// Grid configuration
// =========================
const int GRID_COLS = 3;      // X direction
const int GRID_ROWS = 3;      // Y direction
const double STEP_X = 60.0;   // mm between ducks in X
const double STEP_Y = 60.0;   // mm between ducks in Y

// =========================
// Radians-degrees helpers
// =========================
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

double rad2deg(double r) {
  return r * 180.0 / M_PI;
}

// Forward declaration for debug function (used in moveToXYZ)
void debugTarget(const char* label,
                 double x, double y, double z,
                 double wx, double wy, double wz,
                 double t1, double t2, double t3,
                 double t4, double t5);

// =========================
// IK for 3-DOF position
// Returns true if reachable
// =========================
bool solveIK(double x, double y, double z,
             double &theta1, double &theta2, double &theta3) {
  // Base yaw
  theta1 = atan2(y, x);

  // Radius in XY plane
  double r = sqrt(x*x + y*y);

  // Planar 2-link in (r,z)
  double rp = r;
  double zp = z;

  // Reachability check
  double dist2 = rp*rp + zp*zp;
  double maxReach = (L1 + L2);
  double minReach = fabs(L1 - L2);
  if (dist2 > maxReach*maxReach || dist2 < minReach*minReach) {
    return false;
  }

  // Law of cosines for elbow
  double D = (dist2 - L1*L1 - L2*L2) / (2.0 * L1 * L2);
  if (D > 1.0) D = 1.0;
  if (D < -1.0) D = -1.0;

  // Elbow-down solution
  theta3 = atan2(-sqrt(1.0 - D*D), D);

  // Shoulder
  double phi  = atan2(zp, rp);
  double beta = atan2(L2 * sin(theta3), L1 + L2 * cos(theta3));
  theta2 = phi - beta;

  return true;
}

// =========================
// Compute wrist orientation from arm angles
// theta4 = wrist pitch
// theta5 = wrist rotation/twist
// =========================
void computeWristAngles(double theta1, double theta2, double theta3,
                        double &theta4, double &theta5) {
  // Desired tool pitch: roughly downward in vertical plane
  double desiredToolPitch = -M_PI / 2.0;
  theta4 = desiredToolPitch - (theta2 + theta3);

  // Wrist rotation to keep gripper aligned (no twist for now)
  theta5 = 0.0;
}

// =========================
// Set all 5 joint angles (rad) to servos
// =========================
void setJointAngles(double theta1, double theta2, double theta3,
                    double theta4, double theta5) {

  double baseDeg        = BASE_OFFSET        + BASE_DIR        * rad2deg(theta1);
  double shoulderDeg    = SHOULDER_OFFSET    + SHOULDER_DIR    * rad2deg(theta2);
  double elbowDeg       = ELBOW_OFFSET       + ELBOW_DIR       * rad2deg(theta3);
  double wristPitchDeg  = WRIST_PITCH_OFFSET + WRIST_PITCH_DIR * rad2deg(theta4);
  double wristRotDeg    = WRIST_ROT_OFFSET   + WRIST_ROT_DIR   * rad2deg(theta5);

  // Clamp
  baseDeg       = constrain(baseDeg, 0, 180);
  shoulderDeg   = constrain(shoulderDeg, 0, 180);
  elbowDeg      = constrain(elbowDeg, 0, 180);
  wristPitchDeg = constrain(wristPitchDeg, 0, 180);
  wristRotDeg   = constrain(wristRotDeg, 0, 180);

  // Send to servos
  baseServo.write((int)baseDeg);
  shoulderServo.write((int)shoulderDeg);
  elbowServo.write((int)elbowDeg);
  wristPitchServo.write((int)wristPitchDeg);
  wristRotServo.write((int)wristRotDeg);
}

// =========================
// Smooth motion: move all 5 joints to XYZ (tip)
// =========================
void moveToXYZ(double x, double y, double z,
               int steps = 30, int delayMs = 20) {

  // Convert tip target to wrist target (offset along tool Z)
  double wx = x;
  double wy = y;
  double wz = z + CLAW_LENGTH;

  // Solve IK for wrist
  double t1_target, t2_target, t3_target;
  if (!solveIK(wx, wy, wz, t1_target, t2_target, t3_target)) {
    Serial.println("Target out of reach!");
    return;
  }

  double t4_target, t5_target;
  computeWristAngles(t1_target, t2_target, t3_target, t4_target, t5_target);

  // DEBUG THE TARGET
  debugTarget("MOVE", x, y, z,
                     wx, wy, wz,
                     t1_target, t2_target, t3_target,
                     t4_target, t5_target);

  // Current state (we keep a simple internal memory)
  static double t1_curr = 0.0;
  static double t2_curr = 0.0;
  static double t3_curr = 0.0;
  static double t4_curr = 0.0;
  static double t5_curr = 0.0;

  double d1 = (t1_target - t1_curr) / steps;
  double d2 = (t2_target - t2_curr) / steps;
  double d3 = (t3_target - t3_curr) / steps;
  double d4 = (t4_target - t4_curr) / steps;
  double d5 = (t5_target - t5_curr) / steps;

  for (int i = 1; i <= steps; i++) {
    double t1 = t1_curr + d1 * i;
    double t2 = t2_curr + d2 * i;
    double t3 = t3_curr + d3 * i;
    double t4 = t4_curr + d4 * i;
    double t5 = t5_curr + d5 * i;

    setJointAngles(t1, t2, t3, t4, t5);
    delay(delayMs);
  }

  t1_curr = t1_target;
  t2_curr = t2_target;
  t3_curr = t3_target;
  t4_curr = t4_target;
  t5_curr = t5_target;
}

// =========================
// Gripper helpers
// =========================
void openGripper() {
  gripperServo.write(GRIPPER_OPEN_ANGLE);
  delay(1000);
}

void closeGripper() {
  gripperServo.write(GRIPPER_CLOSE_ANGLE);
  delay(1000);
}
// =========================
// Pick and place sequence
// Now: after box, return to duck offset, then down to duck Z
// =========================
void pickAndPlaceDuck(double dx, double dy, double dz,
                      double bx, double by, double bz) {

  // 1. Above duck (safe height)
  Serial.println("Moving above duck...");
  moveToXYZ(dx, dy, dz + 100);

  // 2. Down to duck
  Serial.println("Moving down to duck...");
  moveToXYZ(dx, dy, dz);
  delay(1000);

  // 3. Close gripper (grab duck)
  Serial.println("Closing gripper...");
  closeGripper();
  delay(500);

  // 4. Lift straight up to safe height
  Serial.println("Lifting straight up...");
  moveToXYZ(dx, dy, dz + 100 );  // extra safety lift
  delay(1000);

  // 5. Move above box
  Serial.println("Moving above box...");
  moveToXYZ(bx, by, bz  );
  delay(1000);

  // 6. Down to box
  Serial.println("Moving down to box...");
  moveToXYZ(bx, by, bz );

  // 7. Release duck in box
  Serial.println("Releasing duck...");
  openGripper();

  // 8. Retreat up from box to safe height
  Serial.println("Retreating up from box...");
  moveToXYZ(bx, by, bz );

  // 9. Come back to duck at offset height
  Serial.println("Returning above duck (offset)...");
  moveToXYZ(dx, dy, dz + LIFT_Z_OFFSET);

  // 10. Drop down to duck Z (with gripper open)
  Serial.println("Dropping back to duck Z...");
  moveToXYZ(dx, dy, dz + LIFT_Z_OFFSET);
}

// =========================
// GRID PICK & PLACE
// 4x4 ducks, spaced 60 mm
// =========================
void runGridPickAndPlace() {

  double startX = duckX;
  double startY = duckY;

  for (int row = 0; row < GRID_ROWS; row++) {
    for (int col = 0; col < GRID_COLS; col++) {

      double px = startX + col * STEP_X;  // move in +X each column
      double py = startY + row * STEP_Y;  // move in +Y each row
      double pz = duckZ;

      Serial.print("=== Duck at row ");
      Serial.print(row);
      Serial.print(", col ");
      Serial.print(col);
      Serial.println(" ===");

      pickAndPlaceDuck(px, py, pz,
                       boxX, boxY, boxZ);

      delay(1500);  // pause between ducks
    }
  }

  Serial.println("GRID PICK & PLACE COMPLETE!");
}

// =========================
// Forward kinematics (for debugging, unused in control)
// =========================
void forwardKinematics(double t1, double t2, double t3,
                       double &x, double &y, double &z) {
  // 2-link planar FK
  double r = L1 * cos(t2) + L2 * cos(t2 + t3);
  z        = L1 * sin(t2) + L2 * sin(t2 + t3);

  // Project to 3D using base rotation t1
  x = r * cos(t1);
  y = r * sin(t1);
}

// =========================
// Debug print for targets and angles
// =========================
void debugTarget(const char* label,
                 double x, double y, double z,
                 double wx, double wy, double wz,
                 double t1, double t2, double t3,
                 double t4, double t5) {

  Serial.println("===========================");
  Serial.print(label); Serial.println(" TARGET DEBUG");
  Serial.println("===========================");

  Serial.print("Tip Target XYZ:     ");
  Serial.print(x); Serial.print(", ");
  Serial.print(y); Serial.print(", ");
  Serial.println(z);

  Serial.print("Wrist Target XYZ:   ");
  Serial.print(wx); Serial.print(", ");
  Serial.print(wy); Serial.print(", ");
  Serial.println(wz);

  Serial.println("--- Joint Angles ---");
  Serial.print("theta1 (base) rad: "); Serial.println(t1, 6);
  Serial.print("theta2 (shoulder) rad: "); Serial.println(t2, 6);
  Serial.print("theta3 (elbow) rad: "); Serial.println(t3, 6);
  Serial.print("theta4 (pitch) rad: "); Serial.println(t4, 6);
  Serial.print("theta5 (rot) rad: "); Serial.println(t5, 6);

  Serial.print("theta1 deg: "); Serial.println(rad2deg(t1));
  Serial.print("theta2 deg: "); Serial.println(rad2deg(t2));
  Serial.print("theta3 deg: "); Serial.println(rad2deg(t3));
  Serial.print("theta4 deg: "); Serial.println(rad2deg(t4));
  Serial.print("theta5 deg: "); Serial.println(rad2deg(t5));

  Serial.println();
}

// =========================
// Arduino setup & loop
// =========================
void setup() {
  Serial.begin(115200);

  baseServo.attach(BASE_PIN);
  shoulderServo.attach(SHOULDER_PIN);
  elbowServo.attach(ELBOW_PIN);
  wristPitchServo.attach(WRIST_PITCH_PIN);
  wristRotServo.attach(WRIST_ROT_PIN);
  gripperServo.attach(GRIPPER_PIN);

  openGripper();

  // Simple "home" pose: all joint angles ~0 in model
  setJointAngles(0.0, M_PI / 2.0, 0.0, 0.0, 0.0);
  delay(1000);

  Serial.println("5DOF arm ready for grid pick & place...");

  Serial.println("Type 'r' in Serial Monitor and press ENTER to start grid pick & place.");

}

void loop() {
  // Hold home pose until user sends command over Serial
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') {
      Serial.println("Command received: RUN GRID PICK & PLACE");
      runGridPickAndPlace();
      Serial.println("Pick and place complete. Returning to HOME pose...");
      setJointAngles(0.0, M_PI / 2.0, 0.0, 0.0, 0.0);  // back to home
    }
  }
}

