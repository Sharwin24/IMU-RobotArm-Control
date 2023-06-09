#include "StepperMotor.hpp"
#include "Constants.hpp"
#include <Arduino.h>
#include <Utils.h>

StepperMotor::StepperMotor(int linkNumber, int stepPin, int dirPin, float outputGearRatio) {
	this->linkNumber = linkNumber;
	this->stepPin = stepPin;
	this->dirPin = dirPin;
	this->outputGearRatio = outputGearRatio;
}

void StepperMotor::init() {
	// Setup pins
	pinMode(stepPin, OUTPUT);
	pinMode(dirPin, OUTPUT);

	// Setup default values
	this->current = 0;
	this->currentAngle = 0.0f;
	this->target = 0;
	this->currentSpeed = LINK_STEPS_PER_SEC;
	this->currentDelay = getDelayFromSpeed(this->currentSpeed);
	this->previousChangeTime = micros();
	this->currentlyRunning = false;
}

void StepperMotor::calibrate() {
	this->current = 0;
	this->currentAngle = 0.0f;
}

int StepperMotor::getCurrent() { return this->current; }
int StepperMotor::getTarget() { return this->target; }
float StepperMotor::getSpeed() { return this->currentSpeed; }
long StepperMotor::getDelay() { return this->currentDelay; }
float StepperMotor::getCurrentAngle() { return this->currentAngle; }

/**
 * @brief Sets the speed [steps per sec] and calculates the required pulse delay
 *
 * @param speed speed in steps per second
 */
void StepperMotor::setSpeed(float speed) {
	this->currentSpeed = speed;
	this->currentDelay = getDelayFromSpeed(this->currentSpeed);
}

/**
 * @brief Sets the direction of the motor's direction pin
 *
 * @param CW True -> CW, False -> CCW
 */
void StepperMotor::setDirection(bool CW) {
	if (CW) {
		digitalWrite(this->dirPin, HIGH);
	} else { // CCW
		digitalWrite(this->dirPin, LOW);
	}
	this->currentDir = CW;
}

/**
 * @brief Sets the target to the given target step. Also sets direction.
 *
 * @note This metod does not move the motor. It only sets the target and direction.
 * 			 This method will not update the target if the motor is currently moving.
 *
 * @param targetStep the absolute target step
 */
void StepperMotor::setTarget(int targetStep) {
	if (this->isMoving()) {
		return; // Motor is currently moving
	}
	this->previous = this->current;
	this->target = targetStep;
	this->setDirection(this->target > this->current); // True -> CW, False -> CCW
}

/**
 * @brief Sets the target to the given target angle. Also sets direction.
 * 			 	This method converts the given target angle to steps and then calls
 * 				setTarget(int targetStep)
 *
 * @note This metod does not move the motor. It only sets the target and direction.
 * 			 This method will not update the target if the motor is currently moving.
 *
 * @param targetAngleDegrees the absolute target angle
 */
void StepperMotor::setTargetAngle(float targetAngleDegrees) {
	int steps = degreeToSteps(targetAngleDegrees);
	this->currentAngle = targetAngleDegrees;
	writeDebug("setTargetAngle [deg,steps] -> Link " + String(this->linkNumber) +
		" [ " + String(targetAngleDegrees) + ", " + String(steps) + " ]");
	setTarget(steps);
}

/**
 * @brief Determines if the motor is moving by comparing the current and target steps
 * @note Because of the implemenetation of gear ratios, we need to use an epsilon
 *
 */
bool StepperMotor::isMoving() {
	return this->outputGearRatio == 1.0f ?
		this->current != this->target :
		abs(this->current - this->target) >= this->outputGearRatio;
}

/**
 * @brief Determines if the motor is at the given angle within the given tolerance
 *
 * @param angle the angle to check [degrees]
 * @param tolerance the tolerance [degrees]
 */
bool StepperMotor::atAngle(float angle, float tolerance) {
	return abs(angle - this->currentAngle) <= tolerance;
}

/**
 * @brief Provides a pulse to the motor using the current delay calculated from the current speed
 *
 */
void StepperMotor::stepMotor() {
	digitalWrite(this->stepPin, HIGH);
	delayMicroseconds(this->currentDelay);
	digitalWrite(this->stepPin, LOW);
	delayMicroseconds(this->currentDelay);
}

/**
 * @brief Primary function to be used for non-blocking motion.
 * This method will be called multiple times per second and will
 * determine when to step the motor given the current time of the program
 * and the time the motor was last pulsed. Updates current and currentAngle
 * as motion occurs.
 *
 */
void StepperMotor::update() {
	// If the motor has reached its target, do nothing
	if (!this->isMoving()) { return; }
	// Determine time since last step
	long currentTime = micros();
	long timeSinceLastStep = currentTime - this->previousChangeTime;
	if (timeSinceLastStep > this->currentDelay) {
		// Swap HIGH/LOW pulse on every occurance of passing the delay
		this->currentlyRunning = !this->currentlyRunning;
		if (this->currentlyRunning) {
			digitalWrite(this->stepPin, HIGH);
		} else {
			digitalWrite(this->stepPin, LOW);
		}
		this->previousChangeTime = currentTime;
		// Speed can be reset here provided an speed curve
		// long newSpeed = getSpeedCurve(stepsMoved, totalSteps);
		// setSpeed(newSpeed);
		if (!this->currentlyRunning) {
			// Increment steps during LOW pulse
			if (this->current < this->target) { // CW
				this->current++;//= (int)this->outputGearRatio;
			} else { // CCW
				this->current--;//= (int)this->outputGearRatio;
			}
		}
	}
}

/**
 * @brief Given a target step (negative steps allowed), calculates the number of steps and direction,
 *        and steps the motor towards the target.
 * @note This is a blocking function and will only allow for this movement to be executed.
 *
 * @param targetStep the target step to acheive. Represents an absolute position
 */
void StepperMotor::moveToTarget(int targetStep) {
	setDirection(targetStep > this->current); // True -> CW, False -> CCW
	int stepsToMove = abs(targetStep - this->current);
	for (int i = 0; i < stepsToMove; i++) {
		stepMotor();
	}
	this->current = targetStep;
	this->target = targetStep;
}

/**
 * @brief A more practical moveTo() function taking in a target angle.
 * This method calls moveTo(int targetStep) after translating the given angle to steps.
 *
 * @param targetAngle the target angle in degrees
 */
void StepperMotor::moveToAngle(float targetAngleDegrees) {
	int targetSteps = degreeToSteps(targetAngleDegrees);
	moveToTarget(targetSteps);
}

/**
 * @brief Calls update() until the current step has met the target step
 *
 * @note This method is blocking and will not return until the motor has reached its target.
 * 			 This method simply runs update() until the current step meets the target step.
 *
 * @return true on function termination
 */
bool StepperMotor::updateToTarget() {
	if (!this->isMoving()) { return true; }
	while (this->isMoving()) { update(); }
	return true;
}

/**
 * @brief Given a speed in steps per second, calculates the delay in microseconds
 * 			  between steps required to achieve that speed
 *
 * @param speed speed of the motor [steps per second]
 * @return long representing the delay in microseconds between steps
 */
long StepperMotor::getDelayFromSpeed(float speed) { return (long)(1000000.0f / speed); }

/**
 * @brief Given an angle in degrees, converts it to a number of steps for the output
 *
 * @param targetAngleDegrees angle to convert to steps to accomplish that angle [0,360)
 * @return long representing the number of steps to accomplish that angle
 */
int StepperMotor::degreeToSteps(float targetAngleDegrees) {
	// SPR * 1 / 360 = steps per degree * degree
	return round((STEPS_PER_REV / 360.0f) * targetAngleDegrees) * this->outputGearRatio;
}