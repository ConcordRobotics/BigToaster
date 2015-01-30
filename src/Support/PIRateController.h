// This class implements a PI (proportional, integral) controller based on rate
// It uses feed-forward to provide a baseline of control.


#ifndef PIRATECONTROLLER_H
#define PIRATECONTROLLER_H
#include "WPILib.h"
#include <string.h>

class PIRateController {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
	void CalcOutput(double delT);
public:
	Timer* timer;
	double curRate;
	double intErr;
	double pGain;
	double iGain;
	double maxOutput;
	double maxRate;
	double target;
	double controlOutput;
	double controlSlope;
	double curTime;
	double lastTime;
	void SetTarget(double targetIn);
    void SetRate(double rate);
    void ResetCont();
    void OutputToDashboard(std::string controllerName);
    PIRateController(double pGain, double iGain, double maxOutputIn,
    		double maxRateIn, double controlSlope);

};
#endif



