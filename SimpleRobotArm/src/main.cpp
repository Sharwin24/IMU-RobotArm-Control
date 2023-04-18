#include <Arduino.h>
#include <RobotArm.hpp>
#include <Utils.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <string.h>

/**
 * @brief Robot Arm object that supports forward and inverse kinematics
 *
 */
RobotArm robotArm = RobotArm();

/**
 * @brief Callback function for the arduino_command topic with
 *        the given message as a String
 *
 * @param msg the arduino command, contained in a string with the format:
 *            "link1AngleDeg,link2AngleDeg,link3AngleDeg"
 */
void arduinoCommandCallback(const std_msgs::String& msg) {
  String input = msg.data;
  writeDebug("Received -> " + input);
  if (robotArm.isMoving()) { return; }
  float link1AngleDeg = input.substring(0, input.indexOf(',')).toFloat();
  float link2AngleDeg = input.substring(input.indexOf(',') + 1, input.lastIndexOf(',')).toFloat();
  float link3AngleDeg = input.substring(input.lastIndexOf(',') + 1, input.length() - 1).toFloat();
  if (robotArm.atConfiguration(link1AngleDeg, link2AngleDeg, link3AngleDeg)) { return; }
  robotArm.forwardKinematics(link1AngleDeg, link2AngleDeg, link3AngleDeg);
}

// Ros objects
// ros::NodeHandle nodeHandler;
// ros::Subscriber<std_msgs::String> sub("/arduino_command", &arduinoCommandCallback);


void serialReactions() {
  writeState("ready");
  waitForSerialInput();
  String input = Serial.readStringUntil('\n');
  writeInfo("Received - " + input);
  if (input.equals("fk")) {
    waitForSerialInput();
    float targetAngle1 = Serial.parseFloat();
    waitForSerialInput();
    float targetAngle2 = Serial.parseFloat();
    waitForSerialInput();
    float targetAngle3 = Serial.parseFloat();
    writeInfo("FK [" + String(targetAngle1) + ", " + String(targetAngle2) + ", " + String(targetAngle3) + "]");
    robotArm.forwardKinematics(targetAngle1, targetAngle2, targetAngle3);
  } else if (input.equals("setSpeed")) {
    waitForSerialInput();
    int linkNumber = Serial.parseInt();
    waitForSerialInput();
    float newSpeedRPM = Serial.parseFloat();
    // Convert RPM to steps/sec
    float newSpeed = STEPS_PER_REV * (newSpeedRPM / 60.0f);
    writeInfo("setSpeed [ Link " + String(linkNumber) + ", " + String(newSpeed) + " steps/sec ]");
    robotArm.setSpeed(linkNumber, newSpeed);
  } else if (input.equals("on")) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else if (input.equals("off")) {
    digitalWrite(LED_BUILTIN, LOW);
  } else {
    writeError("Invalid Command");
  }
}

void setup() {
  Serial.begin(9600);
  robotArm.init();
  robotArm.calibrate();
  pinMode(LED_BUILTIN, OUTPUT);

  // nodeHandler.initNode();
  // nodeHandler.subscribe(sub);
}

void loop() {
  serialReactions();
  // nodeHandler.spinOnce();
  // delay(100);
}