// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#ifndef CLAW_H
#define CLAW_H
#include "Subsystems/LinearSystem.h"
#include "Support/cPIDController.h"
#include "WPILib.h"

/**
 *
 *
 * @author ExampleAuthor
 */

class Claw: public LinearSystem , public Subsystem {
private:

public:
	Claw();
	void EnforceLimits();
	void UpdateController();
	void InitDefaultCommand(void);
	void SetFeedForward();
};

#endif
