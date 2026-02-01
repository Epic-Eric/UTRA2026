// UTRA Hacks 2026 Autonomous Robot Controller
// Board: Arduino Uno (PlatformIO env: uno)
// Kit: 2 DC motors + L298N, 2 Servos (claw + shooter), 2 IR line sensors, Ultrasonic (HC-SR04), Color sensor (TCS3200)
// NOTE: Tune thresholds on-site. See Serial help by opening Serial Monitor at 115200.

#include <Arduino.h>
#include <Servo.h>
#include "pins.hpp"
#include "config.hpp"

// ===================== Pin Mapping =====================
// Moved to pins.hpp; use Pins:: names.

// ===================== Tunable Parameters =====================
// Moved to config.hpp; use Config:: names.

// ===================== Helpers =====================
struct RGBi { unsigned long r; unsigned long g; unsigned long b; };

// Motor control class
class MotorDriver {
public:
  void begin() {
    pinMode(Pins::MOTOR_ENA, OUTPUT);
    pinMode(Pins::MOTOR_IN1, OUTPUT);
    pinMode(Pins::MOTOR_IN2, OUTPUT);
    pinMode(Pins::MOTOR_ENB, OUTPUT);
    pinMode(Pins::MOTOR_IN3, OUTPUT);
    pinMode(Pins::MOTOR_IN4, OUTPUT);
    stop();
  }

  void setLeft(int pwm, bool forward) {
    pwm = constrain(pwm, 0, 255);
    digitalWrite(Pins::MOTOR_IN1, forward ? HIGH : LOW);
    digitalWrite(Pins::MOTOR_IN2, forward ? LOW : HIGH);
    analogWrite(Pins::MOTOR_ENA, pwm);
  }

  void setRight(int pwm, bool forward) {
    pwm = constrain(pwm, 0, 255);
    digitalWrite(Pins::MOTOR_IN3, forward ? HIGH : LOW);
    digitalWrite(Pins::MOTOR_IN4, forward ? LOW : HIGH);
    analogWrite(Pins::MOTOR_ENB, pwm);
  }

  void drive(int leftPwm, int rightPwm) {
    setLeft(abs(leftPwm), leftPwm >= 0);
    setRight(abs(rightPwm), rightPwm >= 0);
  }

  void forward(int pwm) { drive(pwm, pwm); }
  void backward(int pwm) { drive(-pwm, -pwm); }
  void turnLeft(int pwm) { drive(-pwm, pwm); }
  void turnRight(int pwm) { drive(pwm, -pwm); }
  void stop() { drive(0, 0); }
};

// IR line follower
class LineFollower {
public:
  void begin() {
    pinMode(Pins::IR_LEFT, INPUT);
    pinMode(Pins::IR_RIGHT, INPUT);
  }

  int readLeft()  { return analogRead(Pins::IR_LEFT); }
  int readRight() { return analogRead(Pins::IR_RIGHT); }

  // Compute steering error: negative => steer left, positive => steer right
  int error() {
    int l = readLeft();
    int r = readRight();
    // Normalize into -1..1 based on thresholds
    int lDark = l < Config::IR_BLACK_LEVEL ? 1 : 0;
    int rDark = r < Config::IR_BLACK_LEVEL ? 1 : 0;
    return (rDark - lDark); // -1 left is on line, 1 right is on line
  }
};

// Ultrasonic helper
class Ultrasonic {
public:
  void begin() {
    pinMode(Pins::US_TRIG, OUTPUT);
    pinMode(Pins::US_ECHO, INPUT);
    digitalWrite(Pins::US_TRIG, LOW);
  }

  float distanceCM() {
    // Trigger pulse
    digitalWrite(Pins::US_TRIG, LOW);
    delayMicroseconds(4);
    digitalWrite(Pins::US_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(Pins::US_TRIG, LOW);
    // Measure echo
    unsigned long duration = pulseIn(Pins::US_ECHO, HIGH, 30000UL); // timeout ~30ms (~5m)
    if (duration == 0) return 999.0;
    return (duration / 2.0) / 29.1; // speed of sound ~343 m/s
  }
};

// TCS3200 color sensor driver (basic)
class TCS3200 {
public:
  void begin() {
    pinMode(Pins::TCS_S0, OUTPUT);
    pinMode(Pins::TCS_S1, OUTPUT);
    pinMode(Pins::TCS_S2, OUTPUT);
    pinMode(Pins::TCS_S3, OUTPUT);
    pinMode(Pins::TCS_OUT, INPUT);
    // Frequency scaling: S0=HIGH, S1=LOW => 20%
    digitalWrite(Pins::TCS_S0, HIGH);
    digitalWrite(Pins::TCS_S1, LOW);
  }

  // Raw pulse width reading (lower = brighter)
  unsigned long readPulseChannel(bool s2, bool s3) {
    digitalWrite(Pins::TCS_S2, s2 ? HIGH : LOW);
    digitalWrite(Pins::TCS_S3, s3 ? HIGH : LOW);
    delayMicroseconds(100);
    unsigned long period = pulseIn(Pins::TCS_OUT, LOW, 20000UL);
    if (period == 0) period = 20000UL; // fallback
    return period;
  }

  unsigned long readChannel(bool s2, bool s3) {
    // Convert raw pulse to pseudo-intensity (lower pulse => higher intensity)
    unsigned long period = readPulseChannel(s2, s3);
    return 20000UL - period;
  }

  RGBi readRGB() {
    // TCS3200 filter truth table:
    // S2 LOW S3 LOW => Red
    // S2 LOW S3 HIGH => Blue
    // S2 HIGH S3 HIGH => Green
    unsigned long r = readChannel(false, false);
    unsigned long b = readChannel(false, true);
    unsigned long g = readChannel(true, true);
    return {r, g, b};
  }

  // Read raw pulses for calibration-aligned reporting
  RGBi readRGBPulse() {
    unsigned long r = readPulseChannel(false, false);
    unsigned long b = readPulseChannel(false, true);
    unsigned long g = readPulseChannel(true, true);
    return {r, g, b};
  }
};

// ===================== Global Devices =====================
MotorDriver motors;
LineFollower line;
Ultrasonic us;
TCS3200 color;
Servo servoClaw;
Servo servoShoot;

enum ColorClass { COL_UNKNOWN, COL_BLACK, COL_BLUE, COL_RED, COL_GREEN, COL_PURPLE, COL_WHITE };

ColorClass classifyColor(const RGBi &rgb) {
  unsigned long sum = rgb.r + rgb.g + rgb.b;
  if (sum < (unsigned long)Config::COLOR_BLACK_SUM_MAX) return COL_BLACK;

  // Dominance logic
  if (rgb.b > rgb.r * Config::COLOR_BLUE_DOM && rgb.b > rgb.g * Config::COLOR_BLUE_DOM) return COL_BLUE;
  if (rgb.r > rgb.b * Config::COLOR_RED_DOM && rgb.r > rgb.g * Config::COLOR_RED_DOM) return COL_RED;
  if (rgb.g > rgb.r * Config::COLOR_GREEN_DOM && rgb.g > rgb.b * Config::COLOR_GREEN_DOM) return COL_GREEN;

  // Heuristic purple: mix of red + blue high
  if (rgb.r > rgb.g && rgb.b > rgb.g) return COL_PURPLE;

  return COL_WHITE;
}

// Normalize raw channel using calibration min/max into 0..255 range
static int normalizeChannel(unsigned long v, unsigned long vmin, unsigned long vmax) {
  if (vmin >= vmax) return (int)v; // avoid div by zero; fallback raw
  if (v < vmin) v = vmin;
  if (v > vmax) v = vmax;
  long scaled = map((long)v, (long)vmin, (long)vmax, 0L, 255L);
  return (int)scaled;
}

RGBi readRGBNormalized() {
  // Use raw pulse widths and map to 0..255 where 255 is brighter
  RGBi pulse = color.readRGBPulse();
  // Align to calibration style: map(constrain(pulse, MIN, MAX), MIN, MAX, 255, 0)
  unsigned long rn = map((long)constrain(pulse.r, Config::R_MIN, Config::R_MAX), (long)Config::R_MIN, (long)Config::R_MAX, 255L, 0L);
  unsigned long gn = map((long)constrain(pulse.g, Config::G_MIN, Config::G_MAX), (long)Config::G_MIN, (long)Config::G_MAX, 255L, 0L);
  unsigned long bn = map((long)constrain(pulse.b, Config::B_MIN, Config::B_MAX), (long)Config::B_MIN, (long)Config::B_MAX, 255L, 0L);
  return {rn, gn, bn};
}

// Classification using normalized channels (0..255, 255 bright)
ColorClass classifyColorNormalized(const RGBi &n) {
  unsigned long sum = n.r + n.g + n.b;
  // Simple black gate: all channels quite low
  if (n.r < 25 && n.g < 25 && n.b < 25) return COL_BLACK;
  // Dominance checks with margin
  const int M = 20;
  if ((int)n.b - (int)n.r > M && (int)n.b - (int)n.g > M) return COL_BLUE;
  if ((int)n.r - (int)n.g > M && (int)n.r - (int)n.b > M) return COL_RED;
  if ((int)n.g - (int)n.r > M && (int)n.g - (int)n.b > M) return COL_GREEN;
  if (n.r > n.g && n.b > n.g) return COL_PURPLE;
  return COL_WHITE;
}


// ===================== State Machine =====================
enum State {
  ST_STARTUP,
  ST_IDLE,
  ST_SEARCH_BOX,
  ST_PICK_BOX,
  ST_DROP_BOX,
  // Stage 1 revised flow
  ST_FOLLOW_BLACK_TO_SPLIT,
  ST_SELECT_GREEN,
  ST_FOLLOW_GREEN_TO_BLUE_DROP,
  ST_FOLLOW_GREEN_TO_BLACK,
  ST_FOLLOW_BLACK_TO_NONBLACK,
  ST_STRAIGHT_TO_BLACK_AGAIN,
  ST_STAGE1_END,
  // Stage 2
  ST_STAGE2_RUN
};

State state = ST_STARTUP;
unsigned long stateTs = 0;

enum StageMode { STAGE_NONE, STAGE_1, STAGE_2 };
StageMode stageMode = STAGE_NONE;
// Streaming flags
unsigned long irStreamTs = 0;

void setState(State s) {
  state = s;
  stateTs = millis();
  Serial.print("STATE-> "); Serial.println((int)s);
}

// ===================== Servo Actions =====================
void clawOpen()   { servoClaw.write(Config::CLAW_OPEN_POS); }
void clawClose()  { servoClaw.write(Config::CLAW_CLOSED_POS); }
void shooterRest(){ servoShoot.write(Config::SHOOT_REST_POS); }
void shooterFire(){ servoShoot.write(Config::SHOOT_FIRE_POS); delay(250); shooterRest(); }

// ===================== Serial Commands =====================
void printHelp() {
  Serial.println("Commands:");
  Serial.println(" h        : help");
  Serial.println(" ir       : read IR sensors");
  Serial.println(" rgb      : read color sensor");
  Serial.println(" d        : ultrasonic distance (cm)");
  Serial.println(" m f/s/l  : motors forward/stop/slow");
  Serial.println(" c o/c    : claw open/close");
  Serial.println(" shoot    : fire shooter");
  Serial.println(" set bs x : set BASE_SPEED to x");
  Serial.println(" set tg x : set TURN_GAIN to x");
  Serial.println(" set ib x : set IR_BLACK_LEVEL to x");
  Serial.println(" set iw x : set IR_WHITE_LEVEL to x");
  Serial.println(" run1     : start Stage 1 sequence");
  Serial.println(" run2     : start Stage 2 sequence");
  Serial.println(" stop     : stop and idle");
}

void handleSerial() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd == "h") printHelp();
  else if (cmd == "ir") {
    Serial.print("IR L/R: "); Serial.print(line.readLeft()); Serial.print(" / "); Serial.println(line.readRight());
  } else if (cmd == "rgb") {
    RGBi p = color.readRGBPulse();
    Serial.println("Raw pulse widths (lower = brighter):");
    Serial.print("  R="); Serial.print(p.r);
    Serial.print("  G="); Serial.print(p.g);
    Serial.print("  B="); Serial.println(p.b);
  } else if (cmd == "rgbn") {
    RGBi vn = readRGBNormalized();
    Serial.print("RGB(norm) r/g/b: "); Serial.print(vn.r); Serial.print(" / "); Serial.print(vn.g); Serial.print(" / "); Serial.println(vn.b);
    Serial.print("Class(norm): "); Serial.println((int)classifyColorNormalized(vn));
  } else if (cmd == "d") {
    Serial.print("Dist cm: "); Serial.println(us.distanceCM());
  } else if (cmd == "m f") {
    motors.forward(Config::BASE_SPEED);
  } else if (cmd == "m s") {
    motors.stop();
  } else if (cmd == "m l") {
    motors.forward(Config::SLOW_SPEED);
  } else if (cmd == "c o") {
    clawOpen();
  } else if (cmd == "c c") {
    clawClose();
  } else if (cmd == "shoot") {
    shooterFire();
  } else if (cmd == "run1") {
    stageMode = STAGE_1;
    setState(ST_SEARCH_BOX);
  } else if (cmd == "run2") {
    stageMode = STAGE_2;
    setState(ST_STAGE2_RUN);
  } else if (cmd == "stop") {
    stageMode = STAGE_NONE;
    motors.stop();
    setState(ST_IDLE);
  } else if (cmd.startsWith("set ")) {
    // set <var> <value>
    int sp1 = cmd.indexOf(' ');
    int sp2 = cmd.indexOf(' ', sp1 + 1);
    String key = cmd.substring(sp1 + 1, sp2);
    int val = cmd.substring(sp2 + 1).toInt();
    if (key == "bs") Config::BASE_SPEED = val;
    else if (key == "tg") Config::TURN_GAIN = val;
    else if (key == "ib") Config::IR_BLACK_LEVEL = val;
    else if (key == "iw") Config::IR_WHITE_LEVEL = val;
    else if (key == "rmin") Config::R_MIN = val;
    else if (key == "rmax") Config::R_MAX = val;
    else if (key == "gmin") Config::G_MIN = val;
    else if (key == "gmax") Config::G_MAX = val;
    else if (key == "bmin") Config::B_MIN = val;
    else if (key == "bmax") Config::B_MAX = val;
    Serial.print("Updated "); Serial.print(key); Serial.print(" = "); Serial.println(val);
  } else {
    Serial.println("Unknown. type 'h'");
  }
}

// ===================== Behavior Routines =====================
void followLineStep(int speed) {
  int err = line.error();
  int left = speed, right = speed;
  if (err < 0) { // steer left
    left  = speed - Config::TURN_GAIN;
    right = speed + Config::TURN_GAIN;
  } else if (err > 0) { // steer right
    left  = speed + Config::TURN_GAIN;
    right = speed - Config::TURN_GAIN;
  }
  motors.drive(left, right);
}

bool obstacleAhead() {
  return us.distanceCM() < Config::OBSTACLE_STOP_CM;
}

// Seek center black by sweeping until color class becomes COL_BLACK
// removed legacy center-seek helper

// ===================== Setup/Loop =====================
void setup() {
  Serial.begin(115200);
  motors.begin();
  line.begin();
  us.begin();
  color.begin();

  servoClaw.attach(Pins::SERVO_CLAW);
  servoShoot.attach(Pins::SERVO_SHOOT);
  clawOpen();
  shooterRest();

  pinMode(Pins::BTN_CAL, INPUT_PULLUP);
  Serial.println("UTRA 2026 Robot: Ready. Type 'h' for help.");
  Serial.println("Type 'run1' to start Stage 1 or 'run2' for Stage 2.");
  setState(ST_SEARCH_BOX);
}

void loop() {
  handleSerial();

  switch (state) {
    case ST_IDLE: {
      motors.stop();
    } break;

    // ===== Stage 1 Behavior =====
    case ST_SEARCH_BOX: {
      // Follow line until an object is near (blue zone box)
      if (obstacleAhead()) {
        motors.stop();
        setState(ST_PICK_BOX);
      } else {
        followLineStep(Config::BASE_SPEED);
      }
    } break;

    case ST_PICK_BOX: {
      // Close claw to pick box
      clawClose();
      delay(500);
      setState(ST_FOLLOW_BLACK_TO_SPLIT);
    } break;

    case ST_DROP_BOX: {
      // Explicit drop box state (triggered when BLUE is detected on green path)
      clawOpen();
      delay(400);
      setState(ST_FOLLOW_GREEN_TO_BLACK);
    } break;

    case ST_FOLLOW_BLACK_TO_SPLIT: {
      // Follow black line until color sensor sees green or red (crossroad)
      RGBi vn = readRGBNormalized();
      ColorClass c = classifyColorNormalized(vn);
      if (c == COL_GREEN || c == COL_RED) {
        motors.stop();
        setState(ST_SELECT_GREEN);
      } else {
        followLineStep(Config::BASE_SPEED);
      }
    } break;

    case ST_SELECT_GREEN: {
      // Choose green: bias left and search for consistent GREEN readings
      RGBi vn = readRGBNormalized();
      ColorClass c = classifyColorNormalized(vn);
      if (c == COL_GREEN) {
        // Lock in: follow green until blue to drop box
        setState(ST_FOLLOW_GREEN_TO_BLUE_DROP);
      } else {
        // Bias left to acquire green path
        motors.turnLeft(Config::SLOW_SPEED);
      }
    } break;

    case ST_FOLLOW_GREEN_TO_BLUE_DROP: {
      // Follow green path until we see blue, then drop box
      RGBi vn = readRGBNormalized();
      ColorClass c = classifyColorNormalized(vn);
      if (c == COL_BLUE) {
        motors.stop();
        setState(ST_DROP_BOX);
      } else if (c == COL_GREEN) {
        // slight left bias while following green
        motors.drive(Config::BASE_SPEED - 10, Config::BASE_SPEED + 10);
      } else {
        // keep moving, try to reacquire green
        followLineStep(Config::SLOW_SPEED);
      }
    } break;

    case ST_FOLLOW_GREEN_TO_BLACK: {
      // Continue along green until encountering black
      RGBi vn = readRGBNormalized();
      ColorClass c = classifyColorNormalized(vn);
      if (c == COL_BLACK) {
        motors.stop();
        setState(ST_FOLLOW_BLACK_TO_NONBLACK);
      } else {
        // continue along green with slight left bias
        motors.drive(Config::BASE_SPEED - 10, Config::BASE_SPEED + 10);
      }
    } break;

    case ST_FOLLOW_BLACK_TO_NONBLACK: {
      // Follow black line until a different color is seen
      RGBi vn = readRGBNormalized();
      ColorClass c = classifyColorNormalized(vn);
      if (c != COL_BLACK) {
        // switch to going straight until black appears again
        setState(ST_STRAIGHT_TO_BLACK_AGAIN);
      } else {
        followLineStep(Config::BASE_SPEED);
      }
    } break;

    case ST_STRAIGHT_TO_BLACK_AGAIN: {
      // Keep going straight until black is detected again, then stop (Stage 1 end)
      RGBi vn = readRGBNormalized();
      ColorClass c = classifyColorNormalized(vn);
      if (c == COL_BLACK) {
        motors.stop();
        setState(ST_STAGE1_END);
      } else {
        motors.forward(Config::BASE_SPEED);
      }
    } break;

    case ST_STAGE1_END: {
      motors.stop();
      stageMode = STAGE_NONE;
      setState(ST_IDLE);
    } break;

    // legacy states removed

    // ===== Stage 2 Behavior =====
    case ST_STAGE2_RUN: {
      // Read color and follow rules:
      // - if BLUE: turn right
      // - if GREEN: follow with slight left bias
      // - if RED: turn right
      // - if BLACK: go straight
      RGBi vn = readRGBNormalized();
      ColorClass c = classifyColorNormalized(vn);
      if (c == COL_BLUE || c == COL_RED) {
        motors.turnRight(Config::SLOW_SPEED);
      } else if (c == COL_GREEN) {
        motors.drive(Config::BASE_SPEED - 12, Config::BASE_SPEED + 12);
      } else if (c == COL_BLACK) {
        motors.forward(Config::BASE_SPEED);
      } else {
        motors.forward(Config::SLOW_SPEED);
      }
    } break;
  }
}