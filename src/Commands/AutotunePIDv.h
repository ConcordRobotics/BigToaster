#ifndef AutotunePIDV_H
#define AutotunePIDV_H

#include "Support/cPIDController.h"
#include "WPILib.h"

class AutotunePIDv: public Command {
private:
	Subsystem* sys;
	double center;
	double delta;
	double power;

	double target;
	char name[4][8] = {};
	unsigned int count = 0;
	unsigned int countMax = 10;
public:
	cPIDController** controller;
	Encoder** encoder;
	AutotunePIDv(Subsystem* sysin, double dCenter, double dDelta, double powerIn,
			cPIDController* cont[4], Encoder* enc[4], char nameIn[4][4]);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif
