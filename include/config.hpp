// Tunable parameters for UTRA Hacks 2026 robot
#pragma once

namespace Config {
  // Motor speeds
  inline int BASE_SPEED = 150;       // Nominal forward PWM (0-255)
  inline int TURN_GAIN  = 70;        // Additional PWM for steering correction
  inline int SLOW_SPEED = 110;       // Slower speed for precise areas

  // IR thresholds
  inline int IR_BLACK_LEVEL = 400;   // Analog value below this considered "black"
  inline int IR_WHITE_LEVEL = 700;   // Analog value above this considered "white"

  // Ultrasonic
  inline float OBSTACLE_STOP_CM = 15.0;   // Stop if object closer than this

  // Color detection thresholds
  inline int COLOR_BLACK_SUM_MAX = 1200; // Sum(R+G+Blue) below this => black
  inline int COLOR_BLUE_DOM      = 1;    // Dominance multipliers
  inline int COLOR_RED_DOM       = 1;
  inline int COLOR_GREEN_DOM     = 1;

  // Color calibration ranges (copy from on-site calibration)
  inline unsigned long R_MIN = 44;
  inline unsigned long R_MAX = 72;
  inline unsigned long G_MIN = 51;
  inline unsigned long G_MAX = 121;
  inline unsigned long B_MIN = 67;
  inline unsigned long B_MAX = 71;

  // Shooter servo positions
  inline int SHOOT_REST_POS   = 30;
  inline int SHOOT_FIRE_POS   = 110;
  // Claw servo positions
  inline int CLAW_OPEN_POS    = 40;
  inline int CLAW_CLOSED_POS  = 110;
}
