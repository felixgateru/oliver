#include <Arduino.h>
#include "pindef.h"


int L_val, M_val, R_val;

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

  if(M_val == 1){//if the state of middle one is 1, which means detecting black line

     if (L_val == 1 && R_val == 0) { //If a black line is detected on the left, but not on the right, turn left
        left();
    }
     else if (L_val == 0 && R_val == 1) { //Otherwise, if a black line is detected on the right and not on the left, turn right
      right();
    }
     else { //Otherwise,forward
      front();
    }
  }
  else { //No black lines detected in the middle
    if (L_val == 1 && R_val == 0) { //If a black line is detected on the left, but not on the right, turn left
      left();
    }
    else if (L_val == 0 && R_val == 1) { //Otherwise, if a black line is detected on the right and not on the left, turn right
      right();
    }
    else { 
      stop();
    }
  }
}


void front() {
  digitalWrite(left_ctrl, HIGH);
  digitalWrite(right_ctrl, HIGH);
  analogWrite(left_pwm, 155);
  analogWrite(right_pwm, 155);
}

void back() {
  digitalWrite(left_ctrl, LOW);
  digitalWrite(right_ctrl, LOW);
  analogWrite(left_pwm, 155);
  analogWrite(right_pwm, 155);
}

void left() {
  digitalWrite(left_ctrl, LOW);
  digitalWrite(right_ctrl, HIGH);
  analogWrite(left_pwm, 100);
  analogWrite(right_pwm, 155);
}

void right() {
  digitalWrite(left_ctrl, HIGH);
  digitalWrite(right_ctrl, LOW);
  analogWrite(left_pwm, 155);
  analogWrite(right_pwm, 100);
}

void stop() {
  digitalWrite(left_ctrl, LOW);
  analogWrite(left_pwm,0);
  digitalWrite(right_ctrl, LOW);
  analogWrite(right_pwm,0);
}