// This class implements a PI (proportional, integral) controller based on rate
// It uses feed-forward to provide a baseline of control.


#ifndef CSPEEDCONTROLLER_H
#define CSPEEDCONTROLLER_H
#include "WPILib.h"
#include <string.h>


class cSpeedController : public PIDOutput {
protected:
	~cSpeedController();
private:
	bool reversed = false;
public:
	SpeedController* sc;
	void Set(float 	speed);
	void PIDWrite (float speed);
    cSpeedController(SpeedController* scIn, bool reversedIn);
};
#endif



