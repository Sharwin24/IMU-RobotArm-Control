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
 // void arduinoCommandCallback(const std_msgs::String& msg) {
 //   String input = msg.data;
 //   writeDebug("Received -> " + input);
 //   float link1AngleDeg = input.substring(0, input.indexOf(',')).toFloat();
 //   float link2AngleDeg = input.substring(input.indexOf(',') + 1, input.lastIndexOf(',')).toFloat();
 //   float link3AngleDeg = input.substring(input.lastIndexOf(',') + 1, input.length() - 1).toFloat();
 //   robotArm.forwardKinematics(link1AngleDeg, link2AngleDeg, link3AngleDeg);
 // }

 // Ros objects
 // ros::NodeHandle nodeHandler;
 // ros::Subscriber<std_msgs::String> sub("/arduino_command", &arduinoCommandCallback);


void serialReactions() {
  writeState("ready");
  waitForSerialInput();
  String input = Serial.readStringUntil('\n');
  if (input.equals("") || input.equals("\n") || input.equals("\r")) {
    return;
  }
  writeInfo("Received - " + input);
  if (input.equals("fk")) {
    waitForSerialInput();
    String angleString1 = Serial.readStringUntil('\n');
    float targetAngle1 = angleString1.toFloat();
    waitForSerialInput();
    String angleString2 = Serial.readStringUntil('\n');
    float targetAngle2 = angleString2.toFloat();
    waitForSerialInput();
    String angleString3 = Serial.readStringUntil('\n');
    float targetAngle3 = angleString3.toFloat();
    writeInfo("FK [" + String(targetAngle1) + ", " + String(targetAngle2) + ", " + String(targetAngle3) + "]");
    if (robotArm.isMoving()) {
      writeDebug("Already moving");
      return;
    } else if (robotArm.atConfiguration(targetAngle1, targetAngle2, targetAngle3)) {
      writeDebug("Already at configuration");
      return;
    }
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