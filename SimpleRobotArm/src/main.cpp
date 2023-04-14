#include <Arduino.h>
#include <RobotArm.hpp>
#include <Utils.h>

RobotArm robotArm = RobotArm();

void serialReactions() {
  writeState("ready");
  waitForSerialInput();
  String input = Serial.readStringUntil('\n');
  writeInfo("Received - " + input);
  if (input.equals("fk")) {
    writeInfo("Waiting for angles...");
    waitForSerialInput();
    float targetAngle1 = Serial.parseFloat();
    writeInfo("Angle 1: " + String(targetAngle1));
    waitForSerialInput();
    float targetAngle2 = Serial.parseFloat();
    writeInfo("Angle 2: " + String(targetAngle2));
    waitForSerialInput();
    float targetAngle3 = Serial.parseFloat();
    writeInfo("FK [" + String(targetAngle1) + ", " + String(targetAngle2) + ", " + String(targetAngle3) + "]");
    robotArm.forwardKinematics(targetAngle1, targetAngle2, targetAngle3);
  } else if (input.equals("ik")) {
    waitForSerialInput();
    float targetX = Serial.parseFloat();
    waitForSerialInput();
    float targetY = Serial.parseFloat();
    writeInfo("IK [" + String(targetX) + ", " + String(targetY) + "]");
    robotArm.inverseKinematics(targetX, targetY);
  } else if (input.equals("test")) {
    robotArm.linkToAngle(1, 90.0f);
  } else {
    writeError("Invalid Command");
  }
}

void setup() {
  Serial.begin(9600);
  robotArm.init();
  robotArm.calibrate();
}

void loop() {
  serialReactions();
}