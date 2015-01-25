// This class implements a PI (proportional, integral) controller based on rate
// It uses feed-forward to provide a baseline of control.


#ifndef PICONTROLLER_H
#define PICONTROLLER_H
#include "WPILib.h"


class PIController {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
    void FilterRate(double delT);
	void CalcOutput();
public:
	Timer* timer;
	double lastPosition;
	double curPosition;
	double lastRate;
	double curRate;
	double intErr;
	double pGain;
	double iGain;
	double timeFilter;
	double maxOutput;
	double maxRate;
	float target;
	double lastTime;
	double curTime;
	double controlOutput;
	bool rateController;
	void SetTarget(float targetIn);
    void SetPosition(double output);
    void SetRate(double rate);
    PIController(double pGain, double iGain, double timeFilter, double maxOutputIn, double maxRateIn, double initPosition);
};
#endif



