#include <Arduino.h>
#include <RobotArm.hpp>
#include <Utils.h>
#include <ros.h>
#include <msg/ArduinoCommand.msg>

/**
 * @brief Robot Arm object that supports forward and inverse kinematics
 *
 */
RobotArm robotArm = RobotArm();

ros::NodeHandle nodeHandler;
ros::Subscriber<msg::ArduinoCommand> sub("arduino_command", &arduinoCommandCallback);

void arduinoCommandCallback(const msg::ArduinoCommand& msg) {
  if (robotArm.isMoving()) { return; }
  robotArm.forwardKinematics(msg.link1AngleDeg, msg.link2AngleDeg, msg.link3AngleDeg);
}


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
    test();
  } else {
    writeError("Invalid Command");
  }
}

void setup() {
  Serial.begin(9600);
  robotArm.init();
  robotArm.calibrate();

  nodeHandler.initNode();
  // nodeHandler.subscribe();
}

void loop() {
  //serialReactions();
  nodeHandler.spinOnce();
  delay(1);
}