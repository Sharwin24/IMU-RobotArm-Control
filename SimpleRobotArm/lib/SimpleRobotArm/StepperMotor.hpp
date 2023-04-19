#pragma once

class StepperMotor {
public:
	StepperMotor() = default;
	StepperMotor(int linkNumber, int stepPin, int dirPin, float outputGearRatio);

	// Initialize
	void init();
	void calibrate();

	// Getters
	int getCurrent();
	int getTarget();
	float getSpeed();
	long getDelay();
	float getCurrentAngle();
	float getGearRatio();

	// Setters
	void setSpeed(float speed);
	void setDirection(bool CW);
	void setTarget(int targetStep);
	void setTargetAngle(float targetAngleDegrees);

	// State Methods
	bool isMoving();
	bool atAngle(float angleDeg, float tolerance = 1.0f);

	// Movement Methods
	void stepMotor();
	void update();
	void moveToTarget(int targetStep);
	void moveToAngle(float targetAngleDegrees);

	bool updateToTarget();
	// Constructor Arguments
	int linkNumber;
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
	float currentSpeed;
	long currentDelay;
	// Private methods
	int degreeToSteps(float targetAngleDegrees);
	long getDelayFromSpeed(float s);
};