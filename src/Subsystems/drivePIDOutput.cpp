/*
 * drivePIDoutput.cpp
 *
 *  Created on: Jan 17, 2015
 *      Author: concord robotics
 */

#include "drivePIDOutput.h"

drivePIDOutput::drivePIDOutput (float output){
	PIDOutput = output;
}

void drivePIDOutput::PIDWrite(float output)
{
	PIDOutput = output;
}


