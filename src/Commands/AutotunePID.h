#ifndef AutotunePID_H
#define AutotunePID_H

#include "Support/cPIDController.h"
#include "WPILib.h"

class AutotunePID: public Command {
private:
	Subsystem* sys;
	double center;
	double delta;
	double power;

	char* name;
	unsigned int count = 0;
	unsigned int countMax = 10;
public:
	cPIDController* controller;
	Encoder* encoder;
	AutotunePID(Subsystem* sysin, double dCenter, double dDelta, double powerIn,
			cPIDController* cont, Encoder* enc, char* nameIn);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif
