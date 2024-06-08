#include <Arduino.h>
#include "pindef.h"

int L_val, M_val, R_val;
int error, last_error;
float Kp = 1.0; // Proportional gain
float Ki = 0.0; // Integral gain
float Kd = 0.0; // Derivative gain
float P, I, D;
float PID_value;
float motor_speed = 155; // Base speed of the motors

void front();
void back();
void left();
void right();
void stop();
void following();

void setup() {
  Serial.begin(9600);
  pinMode(left_ctrl, OUTPUT);
  pinMode(left_pwm, OUTPUT);
  pinMode(right_ctrl, OUTPUT);
  pinMode(right_pwm, OUTPUT);
  pinMode(sensor_L, INPUT);
  pinMode(sensor_M, INPUT);
  pinMode(sensor_R, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  following();
}

void following() {
  L_val = digitalRead(sensor_L);
  M_val = digitalRead(sensor_M);
  R_val = digitalRead(sensor_R);

  // Calculate error
  if (M_val == 1) {
    if (L_val == 1 && R_val == 0) {
      error = -1; // Left sensor detects line
    } else if (L_val == 0 && R_val == 1) {
      error = 1; // Right sensor detects line
    } else {
      error = 0; // Line is straight
    }
  } else {
    if (L_val == 1 && R_val == 0) {
      error = -2; // Line is more to the left
    } else if (L_val == 0 && R_val == 1) {
      error = 2; // Line is more to the right
    } else {
      error = 0; // No line detected
    }
  }

  // Calculate PID values
  P = error;
  I += error;
  D = error - last_error;
  PID_value = Kp * P + Ki * I + Kd * D;
  last_error = error;

  // Adjust motor speeds based on PID value
  float left_speed = motor_speed - PID_value;
  float right_speed = motor_speed + PID_value;

  analogWrite(left_pwm, constrain(left_speed, 0, 255));
  analogWrite(right_pwm, constrain(right_speed, 0, 255));

  if (PID_value > 0) {
    right();
  } else if (PID_value < 0) {
    left();
  } else {
    front();
  }
}

void front() {
  digitalWrite(left_ctrl, HIGH);
  digitalWrite(right_ctrl, HIGH);
  analogWrite(left_pwm, motor_speed);
  analogWrite(right_pwm, motor_speed);
}

void back() {
  digitalWrite(left_ctrl, LOW);
  digitalWrite(right_ctrl, LOW);
  analogWrite(left_pwm, motor_speed);
  analogWrite(right_pwm, motor_speed);
}

void left() {
  digitalWrite(left_ctrl, LOW);
  digitalWrite(right_ctrl, HIGH);
  analogWrite(left_pwm, motor_speed - abs(PID_value));
  analogWrite(right_pwm, motor_speed + abs(PID_value));
}

void right() {
  digitalWrite(left_ctrl, HIGH);
  digitalWrite(right_ctrl, LOW);
  analogWrite(left_pwm, motor_speed + abs(PID_value));
  analogWrite(right_pwm, motor_speed - abs(PID_value));
}

void stop() {
  digitalWrite(left_ctrl, LOW);
  analogWrite(left_pwm, 0);
  digitalWrite(right_ctrl, LOW);
  analogWrite(right_pwm, 0);
}
