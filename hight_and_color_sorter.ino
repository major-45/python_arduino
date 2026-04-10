#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>

// ── Pin Definitions ────────────────────────────────────────────────────────────
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

#define SERVO_PIN       4

// ── Motion ────────────────────────────────────────────────────────────────────
#define STEPS_PER_MM_XY   5.0
#define STEPS_PER_MM_Z   25.0

#define XY_MAX_SPEED     300
#define XY_ACCELERATION  150
#define Z_MAX_SPEED      200
#define Z_ACCELERATION   100

// ── Claw ──────────────────────────────────────────────────────────────────────
#define CLAW_OPEN    60
#define CLAW_CLOSED  120

// ── Axis Limits ───────────────────────────────────────────────────────────────
#define MAX_X       210.0
#define MIN_X         0.0
#define MAX_Y       250.0
#define MIN_Y         0.0
#define Z_TOP        43.0
#define Z_BOTTOM      0.0

// ── Pick / Drop Z ─────────────────────────────────────────────────────────────
#define PICK_Z    5.0
#define DROP_Z   12.0

// ── Drop Zones ────────────────────────────────────────────────────────────────
#define RED_TALL_X    240.0
#define RED_TALL_Y     10.0
#define RED_SHORT_X   230.0
#define RED_SHORT_Y    10.0

#define GREEN_TALL_X   10.0
#define GREEN_TALL_Y  240.0
#define GREEN_SHORT_X  10.0
#define GREEN_SHORT_Y 230.0

#define BLUE_TALL_X   240.0
#define BLUE_TALL_Y   240.0
#define BLUE_SHORT_X  240.0
#define BLUE_SHORT_Y  230.0

// ── Collision Avoidance ───────────────────────────────────────────────────────
#define SAFE_RADIUS    20.0   // half-width of object footprint (mm)
#define DETOUR_MARGIN   5.0   // extra clearance added to safe radius (mm)
#define MAX_DETOUR_ITER 10    // max waypoint iterations before fallback

// ── Object Storage ────────────────────────────────────────────────────────────
#define MAX_OBJECTS 20

struct Object {
  float x;
  float y;
  float height_mm;
  bool  placed;        // true once picked — excluded from collision checks
  char  color[10];
};

Object objects[MAX_OBJECTS];
int    objectCount = 0;

// ── Steppers & Servo ──────────────────────────────────────────────────────────
AccelStepper stepperX (AccelStepper::DRIVER, X_STEP_PIN,  X_DIR_PIN);
AccelStepper stepperY (AccelStepper::DRIVER, Y_STEP_PIN,  Y_DIR_PIN);
AccelStepper stepperE1(AccelStepper::DRIVER, E1_STEP_PIN, E1_DIR_PIN);
AccelStepper stepperZ (AccelStepper::DRIVER, Z_STEP_PIN,  Z_DIR_PIN);

Servo clawServo;

float currentX = 0.0;
float currentY = 0.0;
float currentZ = Z_TOP;

// ==============================================================================
// HELPERS
// ==============================================================================
long mmToStepsXY(float mm) { return (long)(mm * STEPS_PER_MM_XY); }
long mmToStepsZ (float mm) { return (long)(mm * STEPS_PER_MM_Z);  }

void printPosition() {
  Serial.print("  Pos X:"); Serial.print(currentX);
  Serial.print("mm Y:");    Serial.print(currentY);
  Serial.print("mm Z:");    Serial.print(currentZ);
  Serial.println("mm");
}

// ==============================================================================
// MOTION
// ==============================================================================
void moveZ(float targetMM) {
  targetMM = constrain(targetMM, Z_BOTTOM, Z_TOP);
  stepperZ.moveTo(mmToStepsZ(targetMM));
  Serial.print("  [Z] -> "); Serial.print(targetMM); Serial.println("mm");
  unsigned long t0 = millis();
  while (stepperZ.distanceToGo() != 0) {
    stepperZ.run();
    if (millis() - t0 > 20000) { Serial.println("  ERROR: Z timeout!"); break; }
  }
  currentZ = targetMM;
  delay(300);
}

void moveX(float targetMM) {
  targetMM = constrain(targetMM, MIN_X, MAX_X);
  stepperX.moveTo(mmToStepsXY(targetMM));
  Serial.print("  [X] -> "); Serial.print(targetMM); Serial.println("mm");
  unsigned long t0 = millis();
  while (stepperX.distanceToGo() != 0) {
    stepperX.run();
    if (millis() - t0 > 15000) { Serial.println("  ERROR: X timeout!"); break; }
  }
  currentX = targetMM;
  delay(300);
}

void moveY(float targetMM) {
  targetMM = constrain(targetMM, MIN_Y, MAX_Y);
  long steps = mmToStepsXY(targetMM);
  stepperY.moveTo(steps);
  stepperE1.moveTo(steps);
  Serial.print("  [Y] -> "); Serial.print(targetMM); Serial.println("mm");
  unsigned long t0 = millis();
  while (stepperY.distanceToGo() != 0 || stepperE1.distanceToGo() != 0) {
    stepperY.run();
    stepperE1.run();
    if (millis() - t0 > 15000) { Serial.println("  ERROR: Y timeout!"); break; }
  }
  currentY = targetMM;
  delay(300);
}

// Y first then X — prevents claw pushing adjacent objects
void moveXY(float targetX, float targetY) {
  moveY(targetY);
  moveX(targetX);
}

// ==============================================================================
// CLAW
// ==============================================================================
void openClaw() {
  clawServo.write(CLAW_OPEN);
  Serial.println("  [CLAW] OPEN");
  delay(700);
}

void closeClaw() {
  clawServo.write(CLAW_CLOSED);
  Serial.println("  [CLAW] CLOSED");
  delay(700);
}

// ==============================================================================
// FIX CONFLICTING PINS on RAMPS 1.4
// ==============================================================================
void fixConflictingPins() {
  pinMode(6,  OUTPUT); digitalWrite(6,  LOW);
  pinMode(11, OUTPUT); digitalWrite(11, LOW);
}

// ==============================================================================
// COLLISION AVOIDANCE — point-to-segment distance
// Returns the minimum distance from point P to segment AB.
// ==============================================================================
float pointToSegDist(float px, float py,
                     float ax, float ay,
                     float bx, float by) {
  float dx = bx - ax;
  float dy = by - ay;
  float lenSq = dx*dx + dy*dy;
  if (lenSq < 1e-6f) {
    // Segment is a point
    float ex = px - ax, ey = py - ay;
    return sqrt(ex*ex + ey*ey);
  }
  float t = ((px - ax)*dx + (py - ay)*dy) / lenSq;
  t = constrain(t, 0.0f, 1.0f);
  float closestX = ax + t * dx;
  float closestY = ay + t * dy;
  float ex = px - closestX, ey = py - closestY;
  return sqrt(ex*ex + ey*ey);
}

// ==============================================================================
// COLLISION AVOIDANCE — path blocked check
// Tests the L-shaped path (Y-move then X-move) against all unplaced objects
// except the current target (skipIdx). Sets blockerIdx to the first blocker found.
// ==============================================================================
bool pathIsBlocked(float fromX, float fromY,
                   float toX,   float toY,
                   int skipIdx, int &blockerIdx) {
  float clearance = SAFE_RADIUS + DETOUR_MARGIN;  // 25 mm
  for (int i = 0; i < objectCount; i++) {
    if (i == skipIdx)          continue;
    if (objects[i].placed)     continue;
    float ox = objects[i].x;
    float oy = objects[i].y;
    // Leg 1: Y-move  (fromX, fromY) → (fromX, toY)
    float d1 = pointToSegDist(ox, oy, fromX, fromY, fromX, toY);
    // Leg 2: X-move  (fromX, toY)   → (toX,   toY)
    float d2 = pointToSegDist(ox, oy, fromX, toY,   toX,   toY);
    if (d1 < clearance || d2 < clearance) {
      blockerIdx = i;
      return true;
    }
  }
  return false;
}

// ==============================================================================
// COLLISION AVOIDANCE — navigate with waypoints
// Tries to reach (targetX, targetY) from (currentX, currentY) while avoiding
// unplaced objects. Up to MAX_DETOUR_ITER waypoints are attempted; falls back
// to direct movement if no clear path found.
// ==============================================================================
void navigateTo(float targetX, float targetY, int skipIdx) {
  float fromX = currentX;
  float fromY = currentY;
  int   blocker = -1;

  for (int iter = 0; iter < MAX_DETOUR_ITER; iter++) {
    if (!pathIsBlocked(fromX, fromY, targetX, targetY, skipIdx, blocker)) {
      // Path is clear — move directly
      moveY(targetY);
      moveX(targetX);
      return;
    }

    // Compute two candidate waypoints ±clearance in X from blocker center
    float clearance = SAFE_RADIUS + DETOUR_MARGIN;
    float ox = objects[blocker].x;
    float oy = objects[blocker].y;

    float wpA_X = ox + clearance;
    float wpB_X = ox - clearance;
    float wpY   = fromY;  // stay at current Y for waypoint leg

    // Pick waypoint with shorter total path distance
    float distA = sqrt((wpA_X - fromX)*(wpA_X - fromX)) +
                  sqrt((targetX - wpA_X)*(targetX - wpA_X) +
                       (targetY - wpY)*(targetY - wpY));
    float distB = sqrt((wpB_X - fromX)*(wpB_X - fromX)) +
                  sqrt((targetX - wpB_X)*(targetX - wpB_X) +
                       (targetY - wpY)*(targetY - wpY));

    float wpX = (distA <= distB) ? wpA_X : wpB_X;
    wpX = constrain(wpX, MIN_X, MAX_X);

    Serial.print("  [NAV] Detour via waypoint X=");
    Serial.println(wpX);

    moveX(wpX);   // X detour only — stay at current Y
    fromX = currentX;
    fromY = currentY;
  }

  // Fallback: direct movement after max iterations
  Serial.println("  [NAV] Fallback: direct move");
  moveY(targetY);
  moveX(targetX);
}

// ==============================================================================
// DROP ZONE LOOKUP
// slot 0 = shorter object, slot 1 = taller object
// ==============================================================================
void getDropZone(const char* color, int slot, float &dx, float &dy) {
  if (strcmp(color, "RED") == 0) {
    if (slot == 0) { dx = RED_SHORT_X;   dy = RED_SHORT_Y;   }
    else           { dx = RED_TALL_X;    dy = RED_TALL_Y;    }
  }
  else if (strcmp(color, "GREEN") == 0) {
    if (slot == 0) { dx = GREEN_SHORT_X; dy = GREEN_SHORT_Y; }
    else           { dx = GREEN_TALL_X;  dy = GREEN_TALL_Y;  }
  }
  else if (strcmp(color, "BLUE") == 0) {
    if (slot == 0) { dx = BLUE_SHORT_X;  dy = BLUE_SHORT_Y;  }
    else           { dx = BLUE_TALL_X;   dy = BLUE_TALL_Y;   }
  }
  else {
    dx = RED_SHORT_X; dy = RED_SHORT_Y;  // fallback
  }
}

// ==============================================================================
// PICK AND PLACE
// ==============================================================================
void pickAndPlace(float pickX, float pickY, const char* color, int slot, int objIdx) {
  float dropX, dropY;
  getDropZone(color, slot, dropX, dropY);

  Serial.print("\n>> PICK ");  Serial.print(color);
  Serial.print(" at (");      Serial.print(pickX);
  Serial.print(", ");         Serial.print(pickY);
  Serial.print(") -> slot "); Serial.println(slot == 0 ? "SHORT" : "TALL");

  if (currentZ < Z_TOP - 1.0) moveZ(Z_TOP);

  // Collision-aware approach to pick position
  navigateTo(pickX, pickY, objIdx);
  printPosition();

  openClaw();
  moveZ(PICK_Z);
  printPosition();

  closeClaw();
  delay(400);

  moveZ(Z_TOP);
  // Mark as placed so it is excluded from future collision checks
  objects[objIdx].placed = true;
  printPosition();

  Serial.print(">> PLACE "); Serial.print(color);
  Serial.print(" at (");     Serial.print(dropX);
  Serial.print(", ");        Serial.print(dropY); Serial.println(")");

  // Drop zone is outside the workspace — move Y first then X as usual
  moveY(dropY);
  moveX(dropX);
  printPosition();

  moveZ(DROP_Z);
  printPosition();

  openClaw();
  delay(400);

  moveZ(Z_TOP);
  closeClaw();
  printPosition();

  Serial.println(">> Done.");
}

// ==============================================================================
// SORT — colour priority first, then height ascending within same colour
// ==============================================================================
int colorPriority(const char* color) {
  if (strcmp(color, "RED")   == 0) return 0;
  if (strcmp(color, "GREEN") == 0) return 1;
  if (strcmp(color, "BLUE")  == 0) return 2;
  return 99;
}

void sortByColorThenHeight() {
  for (int i = 0; i < objectCount - 1; i++) {
    for (int j = 0; j < objectCount - i - 1; j++) {
      int pa = colorPriority(objects[j].color);
      int pb = colorPriority(objects[j+1].color);
      bool doSwap = false;
      if (pa > pb) doSwap = true;
      else if (pa == pb && objects[j].height_mm > objects[j+1].height_mm) doSwap = true;
      if (doSwap) {
        Object tmp   = objects[j];
        objects[j]   = objects[j+1];
        objects[j+1] = tmp;
      }
    }
  }
}

// ==============================================================================
// PARSE & STORE
// ==============================================================================
void parseAndStore(String data) {
  data.trim();
  data.replace("\r", "");
  data.replace("\n", "");
  data.replace(" ",  "");
  if (data.length() == 0) return;

  int idx1 = data.indexOf(',');
  int idx2 = (idx1 != -1) ? data.indexOf(',', idx1 + 1) : -1;
  int idx3 = (idx2 != -1) ? data.indexOf(',', idx2 + 1) : -1;

  if (idx1 == -1 || idx2 == -1 || idx3 == -1) {
    Serial.print("ERROR: Bad format [");
    Serial.print(data);
    Serial.println("] Expected: X,Y,Z,COLOR");
    return;
  }

  float  x         = data.substring(0,      idx1).toFloat();
  float  y         = data.substring(idx1+1, idx2).toFloat();
  float  height_mm = data.substring(idx2+1, idx3).toFloat();
  String cStr      = data.substring(idx3+1);
  cStr.trim();
  cStr.toUpperCase();

  if (cStr != "RED" && cStr != "GREEN" && cStr != "BLUE") {
    Serial.print("ERROR: Unknown color [");
    Serial.print(cStr);
    Serial.println("] Valid: RED GREEN BLUE");
    return;
  }

  if (x < MIN_X || x > MAX_X) {
    Serial.print("ERROR: X out of range: "); Serial.println(x);
    return;
  }
  if (y < MIN_Y || y > MAX_Y) {
    Serial.print("ERROR: Y out of range: "); Serial.println(y);
    return;
  }

  if (objectCount >= MAX_OBJECTS) {
    Serial.println("ERROR: Max objects (20) reached!");
    return;
  }

  objects[objectCount].x         = x;
  objects[objectCount].y         = y;
  objects[objectCount].height_mm = height_mm;
  objects[objectCount].placed    = false;
  cStr.toCharArray(objects[objectCount].color, 10);
  objectCount++;

  Serial.print("Stored #");   Serial.print(objectCount);
  Serial.print(": (");        Serial.print(x);
  Serial.print(", ");         Serial.print(y);
  Serial.print(") H:");       Serial.print(height_mm);
  Serial.print("mm Color: "); Serial.println(cStr);
}

// ==============================================================================
// SETUP
// ==============================================================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("=================================");
  Serial.println("  Gantry 210x250mm  Booting...  ");
  Serial.println("=================================");

  pinMode(X_ENABLE_PIN,  OUTPUT); digitalWrite(X_ENABLE_PIN,  LOW);
  pinMode(Y_ENABLE_PIN,  OUTPUT); digitalWrite(Y_ENABLE_PIN,  LOW);
  pinMode(E1_ENABLE_PIN, OUTPUT); digitalWrite(E1_ENABLE_PIN, LOW);
  pinMode(Z_ENABLE_PIN,  OUTPUT); digitalWrite(Z_ENABLE_PIN,  LOW);

  stepperX.setMaxSpeed(XY_MAX_SPEED);
  stepperX.setAcceleration(XY_ACCELERATION);
  stepperX.setCurrentPosition(0);

  stepperY.setMaxSpeed(XY_MAX_SPEED);
  stepperY.setAcceleration(XY_ACCELERATION);
  stepperY.setCurrentPosition(0);

  stepperE1.setMaxSpeed(XY_MAX_SPEED);
  stepperE1.setAcceleration(XY_ACCELERATION);
  stepperE1.setPinsInverted(true, false, false);
  stepperE1.setCurrentPosition(0);

  stepperZ.setMaxSpeed(Z_MAX_SPEED);
  stepperZ.setAcceleration(Z_ACCELERATION);
  stepperZ.setPinsInverted(true, false, false);
  stepperZ.setCurrentPosition(mmToStepsZ(Z_TOP));
  currentZ = Z_TOP;

  fixConflictingPins();
  clawServo.attach(SERVO_PIN);

  Serial.println("Testing claw servo...");
  clawServo.write(CLAW_OPEN);
  delay(800);
  clawServo.write(CLAW_CLOSED);
  delay(800);
  Serial.println("Claw test done. Starting closed.");

  Serial.println("Home: X=0 Y=0 Z=43 | Claw: CLOSED");
  Serial.println("Send: X,Y,Z,COLOR  then  START");
  Serial.println("READY");
}

// ==============================================================================
// LOOP
// ==============================================================================
void loop() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    line.replace("\r", "");

    if (line == "COUNT") {
      Serial.print("Objects stored: ");
      Serial.println(objectCount);
      return;
    }

    if (line == "RESET") {
      objectCount = 0;
      Serial.println("Reset! Objects cleared.");
      Serial.println("READY");
      return;
    }

    if (line == "START") {
      if (objectCount == 0) {
        Serial.println("ERROR: No objects stored!");
        Serial.println("READY");
        return;
      }

      Serial.print("Starting with ");
      Serial.print(objectCount);
      Serial.println(" objects. Sorting RED->GREEN->BLUE then height ascending...");

      sortByColorThenHeight();

      // Reset placed flags after sort so collision avoidance starts fresh
      for (int i = 0; i < objectCount; i++) objects[i].placed = false;

      Serial.println("\n--- Sorted Order ---");
      for (int i = 0; i < objectCount; i++) {
        Serial.print(i+1);                   Serial.print(". ");
        Serial.print(objects[i].color);      Serial.print(" (");
        Serial.print(objects[i].x);          Serial.print(", ");
        Serial.print(objects[i].y);          Serial.print(") H:");
        Serial.print(objects[i].height_mm);  Serial.println("mm");
      }
      Serial.println("--------------------");

      int redSlot = 0, greenSlot = 0, blueSlot = 0;

      for (int i = 0; i < objectCount; i++) {
        int slot = 0;
        if      (strcmp(objects[i].color, "RED")   == 0) slot = redSlot++;
        else if (strcmp(objects[i].color, "GREEN") == 0) slot = greenSlot++;
        else if (strcmp(objects[i].color, "BLUE")  == 0) slot = blueSlot++;

        Serial.print("\n===== Object ");
        Serial.print(i + 1);
        Serial.print(" of ");
        Serial.print(objectCount);
        Serial.println(" =====");
        pickAndPlace(objects[i].x, objects[i].y, objects[i].color, slot, i);
      }

      Serial.println("\n===== Returning Home =====");
      moveZ(Z_TOP);
      moveY(0.0);
      moveX(0.0);
      printPosition();

      Serial.println("\n=================================");
      Serial.println("         ALL DONE!               ");
      Serial.println("=================================");
      Serial.println("DONE");

      objectCount = 0;
      Serial.println("READY");
      return;
    }

    parseAndStore(line);
  }
}
