// This class implements a PI (proportional, integral) controller based on rate
// It uses feed-forward to provide a baseline of control.


#ifndef CPIDCONTROLLER_H
#define CPIDCONTROLLER_H
// The number of past datapoints to save
#include "WPILib.h"
#include "Support/PIDParams.h"
#include <string.h>
#include <iostream>
#include <fstream>
#include <stdio.h>

static const unsigned int nsave = 3;
class cPIDController {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
	static double PIDSampleTime;
	static double setPointAlpha;
	unsigned int mode;
	unsigned int iN, iNM1, iNM2;
	// Gains
	PIDParams* pidParams;
	// Limits
	float setRange[2] = {-1.0, 1.0};
	float dsdtRange[2] = {-1.0, 1.0};
	float outRange[2] = {-1.0, 1.0};
	// Time based variables
	// Use a looping index to prevent having to
	// copy values over
	double setPoint[nsave];
	double output[nsave];
	double dodt[nsave];
	double sensVal[nsave];
	double time[nsave];
	double p =0;
	double i = 0;
	double d = 0;
	double f = 0;;
	PIDSource* pidSource;
	PIDOutput* pidOutput;
	Timer* timer;
	//float rangeOutOverIn;
	//void CalcRangeRatio(void);
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
    void SetRanges(float setRangeIn[2], float dsdtRangeIn[2], float outRangeIn[2]);
    cPIDController(PIDParams* params, PIDSource* pSource, PIDOutput* pOutput);
    void SetPIDParams(PIDParams* pidParamsIn);
    void Reset();
    void SetMode(unsigned int modeIn);
    void LogData(bool active, char* fileName);
};
#endif



