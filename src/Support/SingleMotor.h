// This class implements a PI (proportional, integral) controller based on rate
// It uses feed-forward to provide a baseline of control.


#ifndef SINGLEMOTOR_H
#define SINGLEMOTOR_H
#include "WPILib.h"
#include "PIController.h"
#include <string.h>

class SingleMotor {
private:


public:
	SpeedController* sc;
	PIController* controller;
	Encoder* encoder;
	bool scReversed;
	SingleMotor (SpeedController* scIn, PIController* controllerIn, Encoder* encoderIn);
	void SetPower();
	void UpdateRate();
	void UpdatePosition();
	void OutputToDashboard(std::string motorName);
	void Reset(bool rateCont);
};
#endif



