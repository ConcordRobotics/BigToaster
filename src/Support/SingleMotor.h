// This class implements a PI (proportional, integral) controller based on rate
// It uses feed-forward to provide a baseline of control.


#ifndef SINGLEMOTOR_H
#define SINGLEMOTOR_H
#include "WPILib.h"
#include "PIController.h"


class SingleMotor {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
	float power;
public:
	SpeedController* sc;
	PIController* controller;
	Encoder* encoder;
	bool scReversed;
	SingleMotor (SpeedController* scIn, PIController* controllerIn, Encoder* encoderIn);
	float maxEnc;
	float maxOutput;
	bool PIControlled;
	void SetTarget(float powerIn);
	void SetPower();
	void UpdateController();
};
#endif



