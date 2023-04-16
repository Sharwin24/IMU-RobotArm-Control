#include "RobotArm.hpp"
#include "Constants.hpp"
#include <Arduino.h>
#include <Utils.h>

RobotArm::RobotArm() {
	this->link1 = StepperMotor(1, LINK_1_STEP_PIN, LINK_1_DIR_PIN, 5.0f);
	this->link2 = StepperMotor(2, LINK_2_STEP_PIN, LINK_2_DIR_PIN, 5.0f);
	this->link3 = StepperMotor(3, LINK_3_STEP_PIN, LINK_3_DIR_PIN, 1.0f);
	this->isMoving = false;
}

void RobotArm::init() {
	this->link1.init();
	this->link2.init();
	this->link3.init();
}

void RobotArm::calibrate() {
	this->link1.calibrate();
	this->link2.calibrate();
	this->link3.calibrate();
}

bool RobotArm::isMoving() { return this->isMoving; }

/**
 * @brief Moves all links to their target angles synchronously
 *
 * @param targetAngle1 the target angle of link 1 [deg]
 * @param targetAngle2 the target angle of link 2 [deg]
 * @param targetAngle3 the target angle of link 3 [deg]
 */
void RobotArm::forwardKinematics(float targetAngle1, float targetAngle2, float targetAngle3) {
	this->isMoving = true;
	// Set all target angles
	this->link1.setTargetAngle(targetAngle1);
	this->link2.setTargetAngle(targetAngle2);
	this->link3.setTargetAngle(targetAngle3);
	writeInfo("L1 target: " + String(this->link1.getTarget()) + " speed: " + String(this->link1.getSpeed()));
	writeInfo("L2 target: " + String(this->link2.getTarget()) + " speed: " + String(this->link2.getSpeed()));
	writeInfo("L3 target: " + String(this->link3.getTarget()) + " speed: " + String(this->link3.getSpeed()));
	// Update all links until they are at their target
	while (this->link1.isMoving() || this->link2.isMoving() || this->link3.isMoving()) {
		this->link1.update();
		this->link2.update();
		this->link3.update();
	}
	this->isMoving = false;
	writeInfo("L1: " + String(this->link1.getCurrent()) + " L2: " + String(this->link2.getCurrent()) + " L3: " + String(this->link3.getCurrent()));
	writeInfo("Movement complete");
}

/**
 * @brief Calculates the inverse kinematics of the robot arm
 *
 * @param targetX the target x position [mm]
 * @param targetY the target y position [mm]
 */
void RobotArm::inverseKinematics(float targetX, float targetY) {
	float ell = sqrtf(sq(targetX) + sq(targetY));
	float q2Rad = -acosf(
		(sq(ell) - sq(LINK_1_LENGTH) - sq(LINK_2_LENGTH)) /
		(2.0f * LINK_1_LENGTH * LINK_2_LENGTH)
	);
	float beta = acosf(
		(sq(ell) + sq(LINK_1_LENGTH) - sq(LINK_2_LENGTH)) /
		(2.0f * ell * LINK_1_LENGTH)
	);
	float alpha = atan2f(targetY, targetX);
	float q1Rad = alpha - beta; // + for elbow up, - for elbow down
	float phi = 270.0f; // Link 3 desired angle [deg]
	float q3Rad = phi - q1Rad - q2Rad;
	float q1Deg = degrees(q1Rad);
	float q2Deg = degrees(q2Rad);
	float q3Deg = degrees(q3Rad);
	// Move the links after computation
	this->forwardKinematics(q1Deg, q2Deg, q3Deg);
}

void RobotArm::linkToAngle(int linkNumber, float targetAngle) {
	switch (linkNumber) {
	case 1:
		this->link1.moveToAngle(targetAngle);
		break;
	case 2:
		this->link2.moveToAngle(targetAngle);
		break;
	case 3:
		this->link3.moveToAngle(targetAngle);
		break;
	default:
		break;
	}
}