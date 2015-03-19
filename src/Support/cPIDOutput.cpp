/*
 * singleMotor.cpp
 *
 *  Created on: Jan 17, 2015
 *      Author: concord robotics
 */

#include "cPIDOutput.h"
cPIDOutput::cPIDOutput () {
	output = 0.0;
}

float cPIDOutput::Get(void) {
	return output;
}

void cPIDOutput::PIDWrite(float out) {
	output = out;
}

cPIDOutput::~cPIDOutput(void) {

}
