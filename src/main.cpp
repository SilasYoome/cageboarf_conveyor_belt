#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "PCA9685.h"
#include "SimpleKalmanFilter.h"
#include "Pixetto.h"

#define sevo1_pin 0
#define servo1_angle_min -90
#define servo1_angle_max 90
#define servo1_angle_init -45

#define sevo2_pin 1
#define servo2_angle_min -90
#define servo2_angle_max 90
#define servo2_angle_init 0

#define sevo3_pin 2
#define servo3_angle_min -90
#define servo3_angle_max 90
#define servo3_angle_init 0

#define sevo4_pin 3
#define servo4_angle_min -90
#define servo4_angle_max 90
#define servo4_angle_init 0

#define rxPin 11
#define txPin 9

Pixetto ss(rxPin, txPin);

PCA9685 driver;
// PCA9685 輸出 = 12 位 = 4096 步
// 20ms 的 2.5% = 0.5ms ; 20ms 的 12.5% = 2.5ms
// 4096 的 2.5% = 102 步；4096 的 12.5% = 512 步
PCA9685_ServoEval pwmServo1; // (0deg, 90deg, 180deg)
PCA9685_ServoEval pwmServo2; // (0deg, 90deg, 180deg)

short color_id;
short door_state = 1;
bool run_finish = 1;
long time;

void forward();
void stop2();
void backward();
void right();

void setup() {
  Serial.begin(9600);
  pinMode(7, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(6, OUTPUT);
  Wire.begin(); // Wire must be started first

  driver.resetDevices(); // Software resets all PCA9685 devices on Wire line
  driver.init(); // Address pins A5-A0 set to B000000
  driver.setPWMFreqServo(); // Set frequency to 50Hz

  driver.setChannelPWM(0, pwmServo1.pwmForAngle(servo1_angle_init));
  delay(10);
  driver.setChannelPWM(1, pwmServo2.pwmForAngle(servo2_angle_init));
  delay(10);

  ss.begin();
  ss.enableFunc(Pixetto::FUNC_COLOR_DETECTION);
  delay(10);
  Serial.begin(9600);
  forward();
}

void loop() {
  if (ss.isDetected()) {
    if (ss.getFuncID() == Pixetto::FUNC_COLOR_DETECTION) {
      if (ss.getTypeID() == Pixetto::COLOR_RED) {
        Serial.println("red");
        if(run_finish){
          color_id = Pixetto::COLOR_RED;
          run_finish = 0;
        }
      }
      else if (ss.getTypeID() == Pixetto::COLOR_YELLOW) {
        Serial.println("yellow");
        if(run_finish){
          color_id = Pixetto::COLOR_YELLOW;
          run_finish = 0;
        }
      }
    }
  }

  if(run_finish == 0){
    if (color_id == Pixetto::COLOR_RED)
    {
      driver.setChannelPWM(1, pwmServo2.pwmForAngle(-10));
      color_id = 0;
    }else if(color_id == Pixetto::COLOR_YELLOW)
    {
      driver.setChannelPWM(1, pwmServo2.pwmForAngle(45));
      color_id = 0;
    }else{
      if(door_state == 1){
        time = millis();
        driver.setChannelPWM(0, pwmServo1.pwmForAngle(20));
        door_state = 2;
      }
      else if(door_state == 2){
        if(millis() - time >= 1500){
          driver.setChannelPWM(0, pwmServo1.pwmForAngle(servo1_angle_init));
          time = millis();
          door_state = 3;
        }
      }else{
        if(millis() - time >= 1500){
          door_state = 1;
          run_finish = 1;
        }
      }
    }
  }

}

void forward() {
  digitalWrite(7, LOW);
  analogWrite(5, 150);
  digitalWrite(4, HIGH);
  analogWrite(6, 250);
}

void stop2() {
  analogWrite(5, 0);
  analogWrite(6, 0);
}

void left() {
  digitalWrite(7, LOW);
  analogWrite(5, 0);
  digitalWrite(4, HIGH);
  analogWrite(6, 250);
}

void backward() {
  digitalWrite(7, HIGH);
  analogWrite(5, 250);
  digitalWrite(4, LOW);
  analogWrite(6, 250);
}

void right() {
  digitalWrite(7, LOW);
  analogWrite(5, 250);
  digitalWrite(4, LOW);
  analogWrite(6, 0);
}