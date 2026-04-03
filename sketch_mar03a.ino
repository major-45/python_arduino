#include <AccelStepper.h>
#include <Servo.h>

#define X_STEP_PIN     54
#define X_DIR_PIN      55
#define X_ENABLE_PIN   38

#define Y_STEP_PIN     60
#define Y_DIR_PIN      61
#define Y_ENABLE_PIN   56

#define E1_STEP_PIN    36
#define E1_DIR_PIN     34
#define E1_ENABLE_PIN  30

#define Z_STEP_PIN     46
#define Z_DIR_PIN      48
#define Z_ENABLE_PIN   62

#define SERVO_PIN      11

// Full step, no jumpers on A4988
#define STEPS_PER_MM_XY   5.0     // 200 steps / 40mm
#define STEPS_PER_MM_Z    25.0    // 200 steps / 8mm

#define XY_MAX_SPEED      200      // steps/sec
#define XY_ACCELERATION   40      // steps/sec2
#define Z_MAX_SPEED        200     // steps/sec
#define Z_ACCELERATION     20     // steps/sec2

#define CLAW_OPEN    70
#define CLAW_CLOSED  130

AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper stepperE1(AccelStepper::DRIVER, E1_STEP_PIN, E1_DIR_PIN);
AccelStepper stepperZ(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);

Servo clawServo;

float currentX = 0.0;
float currentY = 0.0;
float currentZ = 0.0;

long mmToStepsXY(float mm) {
  return (long)(mm * STEPS_PER_MM_XY);
}

long mmToStepsZ(float mm) {
  return (long)(mm * STEPS_PER_MM_Z);
}

void printPosition() {
  Serial.print("  Position -> X: "); Serial.print(currentX);
  Serial.print(" mm | Y: "); Serial.print(currentY);
  Serial.print(" mm | Z: "); Serial.print(currentZ);
  Serial.println(" mm");
}

void moveY(float targetMM) {
  long steps = mmToStepsXY(targetMM);
  stepperY.moveTo(steps);
  stepperE1.moveTo(-steps);
  Serial.print("  [Y] Moving to: "); Serial.print(targetMM); Serial.println(" mm");
  while (stepperY.distanceToGo() != 0 || stepperE1.distanceToGo() != 0) {
    stepperY.run();
    stepperE1.run();
  }
  currentY = targetMM;
  Serial.println("  [Y] Done.");
  delay(300);
}

void moveX(float targetMM) {
  stepperX.moveTo(mmToStepsXY(targetMM));
  Serial.print("  [X] Moving to: "); Serial.print(targetMM); Serial.println(" mm");
  while (stepperX.distanceToGo() != 0) {
    stepperX.run();
  }
  currentX = targetMM;
  Serial.println("  [X] Done.");
  delay(300);
}

void moveZ(float targetMM) {
  stepperZ.moveTo(mmToStepsZ(targetMM));
  Serial.print("  [Z] Moving to: "); Serial.print(targetMM); Serial.println(" mm");
  while (stepperZ.distanceToGo() != 0) {
    stepperZ.run();
  }
  currentZ = targetMM;
  Serial.println("  [Z] Done.");
  delay(300);
}

void moveXY(float targetX, float targetY) {
  Serial.println(">> XY Move: Y first, then X");
  moveY(targetY);
  moveX(targetX);
  printPosition();
}

void openClaw() {
  clawServo.write(CLAW_OPEN);
  Serial.println("  [CLAW] OPEN (70 deg)");
  delay(600);
}

void closeClaw() {
  clawServo.write(CLAW_CLOSED);
  Serial.println("  [CLAW] CLOSED (130 deg)");
  delay(600);
}

void setup() {
  Serial.begin(115200);
  Serial.println("=================================");
  Serial.println("    Gantry Pick & Place Init     ");
  Serial.println("=================================");
  Serial.println("Steps/mm XY: 5  |  Steps/mm Z: 25");
  Serial.println("Microstepping: FULL STEP (no jumpers)");

  pinMode(X_ENABLE_PIN,  OUTPUT);  digitalWrite(X_ENABLE_PIN,  LOW);
  pinMode(Y_ENABLE_PIN,  OUTPUT);  digitalWrite(Y_ENABLE_PIN,  LOW);
  pinMode(E1_ENABLE_PIN, OUTPUT);  digitalWrite(E1_ENABLE_PIN, LOW);
  pinMode(Z_ENABLE_PIN,  OUTPUT);  digitalWrite(Z_ENABLE_PIN,  LOW);

  stepperX.setMaxSpeed(XY_MAX_SPEED);
  stepperX.setAcceleration(XY_ACCELERATION);
  stepperX.setCurrentPosition(0);

  stepperY.setMaxSpeed(XY_MAX_SPEED);
  stepperY.setAcceleration(XY_ACCELERATION);
  stepperY.setCurrentPosition(0);

  stepperE1.setMaxSpeed(XY_MAX_SPEED);
  stepperE1.setAcceleration(XY_ACCELERATION);
  stepperE1.setCurrentPosition(0);

  stepperZ.setMaxSpeed(Z_MAX_SPEED);
  stepperZ.setAcceleration(Z_ACCELERATION);
  stepperZ.setCurrentPosition(0);

  clawServo.attach(SERVO_PIN);
  closeClaw();

  Serial.println("All axes at (0,0,0) | Claw: CLOSED");
  delay(1500);

  Serial.println("\n===== STEP 1: Move to object at (30, -50) =====");
  moveXY(30.0, -50.0);

  Serial.println("\n===== STEP 2: Open claw =====");
  openClaw();

  Serial.println("\n===== STEP 3: Lower Z to 35mm =====");
  moveZ(35.0);
  printPosition();

  Serial.println("\n===== STEP 4: Close claw (pick object) =====");
  closeClaw();

  Serial.println("\n===== STEP 5: Raise Z to 0mm =====");
  moveZ(0.0);
  printPosition();

  Serial.println("\n===== STEP 6: Move to drop position (0, -70) =====");
  moveXY(0.0, -70.0);

  Serial.println("\n===== STEP 7: Lower Z to 35mm =====");
  moveZ(35.0);
  printPosition();

  Serial.println("\n===== STEP 8: Open claw (place object) =====");
  openClaw();

  Serial.println("\n===== STEP 9: Raise Z to 0mm =====");
  moveZ(0.0);
  printPosition();

  Serial.println("\n===== STEP 10: Close claw =====");
  closeClaw();

  Serial.println("\n===== STEP 11: Move to final position (20, -40) =====");
  moveXY(20.0, -40.0);

  Serial.println("\n=================================");
  Serial.println("    Pick & Place COMPLETE!       ");
  Serial.println("=================================");
}

void loop() {
}
