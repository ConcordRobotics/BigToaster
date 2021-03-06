// This class implements a PI (proportional, integral) controller based on rate
// It uses feed-forward to provide a baseline of control.


#ifndef CPIDCONTROLLER_H
#define CPIDCONTROLLER_H
// The number of past datapoints to save
#define NSAVE 2
#include "WPILib.h"
#include <string.h>
#include <iostream>
#include <fstream>
#include <stdio.h>

static const unsigned int nsave = 2;
class cPIDController {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
	static double PIDSampleTime;
	static double setPointAlpha;
	unsigned int mode;
	unsigned int ind;
	// Gains
	float pGain;
	float iGain;
	float dGain;
	float fGain;
	// Limits
	float outRange[2] = {-1.0, 1.0};
	float inRange[2] = {-1.0, 1.0};
	bool smoothReset = false;
	// Time based variables
	// Use a looping index to prevent having to
	// copy values over
	double setPoint[nsave];
	double output[nsave];
	double dodt[nsave];
	double sensVal[nsave];
	double time[nsave];
	double intErr;
	PIDSource* pidSource;
	PIDOutput* pidOutput;
	Timer* timer;
	float rangeOutOverIn;
	void CalcRangeRatio(void);
	bool logData = false;
	std::ofstream logFile;
	_IO_FILE* cLogFile;
public:
	enum modeType {OFF, ENABLED, DIRECT};
	double GetSetpoint();
	void SetSetpoint(double set);
	void SmoothReset(double out, double set);
	void UpdateController(double ff);
    void OutputToDashboard(std::string controllerName);
    void SetInputRange(float iMin, float iMax);
    void SetOutputRange(float oMin, float oMax);
    cPIDController(float p, float i, float d, float f, PIDSource* pSource, PIDOutput* pOutput);
    void Reset();
    void SetMode(unsigned int modeIn);
    void LogData(bool active, char* fileName);
};
#endif



