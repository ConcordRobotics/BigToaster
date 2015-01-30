// This class implements a PI (proportional, integral) controller based on rate
// It uses feed-forward to provide a baseline of control.


#ifndef SINGLEMOTOR_H
#define SINGLEMOTOR_H
#include "WPILib.h"
#include <string.h>
#include <Support/PIRateController.h>

class SingleMotor {
private:


public:
	SpeedController* sc;
	PIRateController* controller;
	Encoder* encoder;
	bool scReversed;
	SingleMotor (SpeedController* scIn, PIRateController* controllerIn, Encoder* encoderIn);
	void SetPower();
	void UpdateRate();
	void OutputToDashboard(std::string motorName);
	void Reset(bool rateCont);
};
#endif



