// This class implements a PI (proportional, integral) controller based on rate
// It uses feed-forward to provide a baseline of control.


#ifndef CPIDCONTROLLER_H
#define CPIDCONTROLLER_H
// The number of past datapoints to save
#include "WPILib.h"
#include "Support/PIDParams.h"
#include "Support/ControllerLimits.h"
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
	ControllerLimits* lim;
	// Time based variables
	// Use a looping index to prevent having to
	// copy values over
	double setPoint[nsave];
	double dSensDt[nsave];
	double sensVal[nsave];
	double time[nsave];
	double output = 0.0;
	double p = 0;
	double i = 0;
	double d = 0;
	double f = 0;
	double rate = 0.0;
	PIDSource* pidSource;
	PIDOutput* pidOutput;
	Timer* timer;
	//float rangeOutOverIn;
	//void CalcRangeRatio(void);
	bool logData = false;
	std::string logName;
    void CheckLimits(double delT);
    void ApplyRate(double delT);
public:
	enum modeType {OFF, RATE, POSITION, DIRECT};
	double GetSetpoint();
	void SetSetpoint(double setIn);
	void SetRate(double rateIn);
	void SetFeedForward(double fIn);
	double UpdateController(double curOutput);
    void OutputToDashboard(std::string controllerName);
    void SetRanges(float setRangeIn[2], float dsdtRangeIn[2], float outRangeIn[2]);
    cPIDController(PIDParams* params, ControllerLimits* pLim, PIDSource* pSource, PIDOutput* pOutput);
    void SetPIDParams(PIDParams* params);
    void Reset();
    void SetMode(int modeIn);
    void LogData(bool active, char* fileName);
};
#endif



