// Pin assignments for UTRA Hacks 2026 robot
#pragma once
#include <Arduino.h>

namespace Pins {
  // Motor driver (L298N style)
  constexpr uint8_t MOTOR_ENA = 5;   // Left motor speed (PWM)
  constexpr uint8_t MOTOR_IN1 = 7;   // Left motor direction
  constexpr uint8_t MOTOR_IN2 = 8;   // Left motor direction
  constexpr uint8_t MOTOR_ENB = 6;   // Right motor speed (PWM)
  constexpr uint8_t MOTOR_IN3 = 11;  // Right motor direction
  constexpr uint8_t MOTOR_IN4 = 12;  // Right motor direction

  // Servos
  constexpr uint8_t SERVO_CLAW  = 9;   // Claw open/close
  constexpr uint8_t SERVO_SHOOT = 10;  // Shooter flicker/launcher

  // IR line sensors (analog)
  constexpr uint8_t IR_LEFT  = A0;
  constexpr uint8_t IR_RIGHT = A1;

  // Ultrasonic HC-SR04
  constexpr uint8_t US_TRIG = A2;
  constexpr uint8_t US_ECHO = A3;

  // Color sensor TCS3200 (S0,S1,S2,S3,OUT)
  constexpr uint8_t TCS_S0  = 2;
  constexpr uint8_t TCS_S1  = 3;
  constexpr uint8_t TCS_S2  = 4;
  constexpr uint8_t TCS_S3  = 13;
  constexpr uint8_t TCS_OUT = A4;

  // User button (optional) for quick calibration trigger
  constexpr uint8_t BTN_CAL = A5;
}
