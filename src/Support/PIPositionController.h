// This class implements a PI (proportional, integral) controller based on rate
// It uses feed-forward to provide a baseline of control.


#ifndef PIPOSITIONCONTROLLER_H
#define PIPOSITIONCONTROLLER_H
#include "WPILib.h"
#include <string.h>

class PIPositionController {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
	void CalcOutput(double delT);
public:
	Encoder* encoder;
	Timer* timer;
	double lastPosition;
	double curPosition;
	double intErr;
	double pGain;
	double iGain;
	double maxOutput;
	double target;
	double lastTime;
	double curTime;
	double controlOutput;
	void SetTarget(double targetIn);
    void UpdateController();
    void Reset();
    void OutputToDashboard(std::string controllerName);
    PIPositionController(double pGain, double iGain, double maxOutputIn, Encoder* encoderIn);

};
#endif



