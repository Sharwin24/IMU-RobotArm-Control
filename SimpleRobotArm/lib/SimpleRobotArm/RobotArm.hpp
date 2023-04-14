#pragma once
#include "StepperMotor.hpp"
#include "Constants.hpp"

class RobotArm {
public:
	RobotArm();

	// Initialize
	void init();
	void calibrate();

	// Async Movement operations (non-blocking)
	void forwardKinematics(float targetAngle1, float targetAngle2, float targetAngle3);
	void inverseKinematics(float targetX, float targetY);

	// Sync Movement operations (blocking)
	void linkToAngle(int linkNumber, float targetAngle);

private:
	StepperMotor link1;// = StepperMotor(LINK_1_STEP_PIN, LINK_1_DIR_PIN, 5.0f);
	StepperMotor link2;// = StepperMotor(LINK_2_STEP_PIN, LINK_2_DIR_PIN, 5.0f);
	StepperMotor link3;// = StepperMotor(LINK_3_STEP_PIN, LINK_3_DIR_PIN, 1.0f);
};