

#ifndef CONTROLLER_LIMITS_H_
#define CONTROLLER_LIMITS_H_

class ControllerLimits {
public:
	double pMin;
	double pMax;
	double rMin;
	double rMax;
	double oMin;
	double oMax;
	double pRange;
	ControllerLimits (double pMinIn, double pMaxIn, double rMinIn, double rMaxIn, double oMinIn, double oMaxIn);
	double ApplyRateLimits(double rate);
	double ApplyPositionLimits(double pos);
	double ApplyOutputLimits(double output);
};




#endif
