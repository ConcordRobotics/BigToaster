


#ifndef CPIDOUTPUT_J
#define CPIDOUTPUT_J
#include "WPILib.h"
#include <string.h>


class cPIDOutput : public PIDOutput {
protected:
	~cPIDOutput();
private:
	float output = 0.0;
public:
	cPIDOutput();
	float Get(void);
	void PIDWrite (float out);
};
#endif



