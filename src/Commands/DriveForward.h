// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#ifndef DRIVEFORWARD_H
#define DRIVEFORWARD_H


#include "Commands/Subsystem.h"
#include "RobotMap.h"
#include "Robot.h"
#include "Support/cPIDController.h"

/**
 *
 *
 * @author ExampleAuthor
 */
class DriveForward: public Command {
private:
	float heading;
	double distance;
	double wheelStart[4] = {0.0, 0.0, 0.0, 0.0};
	double tol;
	float rate;
	double totDist = 0.0;
	double pGain = 1.0;
public:
	DriveForward(float rateIn, double disIn, double tolIn, float angleIn);
	virtual void Initialize();
	virtual void Execute();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();
};

#endif
