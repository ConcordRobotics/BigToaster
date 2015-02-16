// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#ifndef CLAWGRIP_H
#define CLAWGRIP_H

#include "../Robot.h"
#include "Subsystems/LinearSystem.h"
#include "Commands/LinearSysPosition.h"

/**
 *
 *
 * @author ExampleAuthor
 */
class ClawGrip: public LinearSysPosition {
private:
	Timer* timer;
	double rTol;
public:
	ClawGrip(double rateTol);
	virtual bool IsFinished();
};

#endif
