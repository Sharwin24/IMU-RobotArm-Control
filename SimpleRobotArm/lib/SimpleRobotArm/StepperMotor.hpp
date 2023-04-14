#pragma once

class StepperMotor {
public:
	StepperMotor() = default;
	StepperMotor(int stepPin, int dirPin, float outputGearRatio);

	// Initialize
	void init();
	void calibrate();

	// Getters
	float getTarget();
	float getSpeed();
	long getDelay();
	float getCurrentAngle();
	float getGearRatio();

	// Setters
	void setSpeed(float speed);
	void setDirection(bool CW);
	void setTarget(int targetStep);
	void setTargetAngle(float targetAngleDegrees);
	void setCurrentAngle(float currentAngleDegrees);

	// State Methods
	bool isMoving();

	// Movement Methods
	void stepMotor();
	void update();
	void moveToTarget(int targetStep);
	void moveToAngle(float targetAngleDegrees);

	bool updateToTarget();
	// Constructor Arguments
	int stepPin;
	int dirPin;
	float outputGearRatio;
	// Control Variables
	bool currentDir; // True -> CW, False -> CCW
	int current;
	float currentAngle;
	int target;
	int previous;
	long previousChangeTime;
	bool currentlyRunning;
	long maxSpeed;
	float currentSpeed;
	long currentDelay;
	// Private methods
	void setAngle();
	int degreeToSteps(float targetAngleDegrees);
	long getDelayFromSpeed(float s);
};