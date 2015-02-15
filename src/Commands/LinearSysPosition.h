// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#ifndef LINEARSYSPOSITION_H
#define LINEARSYSPOSITION_H

#include "../Robot.h"
#include "Subsystems/LinearSystem.h"

/**
 *
 *
 * @author ExampleAuthor
 */
class LinearSysPosition: public Command {
protected:
	double position;
	LinearSystem* sys;
	double tolerance; // percent tolerance to find position
public:
	LinearSysPosition(Subsystem* sysIn,  LinearSystem* linSys, double position, double tol = -1.0);
	virtual void Initialize();
	virtual void Execute();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();
};

#endif
