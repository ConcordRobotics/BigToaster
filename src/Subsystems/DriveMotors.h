// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#ifndef DRIVEMOTORS_H
#define DRIVEMOTORS_H
#include "Commands/Subsystem.h"
#include "WPILib.h"
#include "drivePIDOutput.h"

/**
 *
 *
 * @author ExampleAuthor
 */
class DriveMotors: public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
public:
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	SpeedController* frontLeftSC;
	SpeedController* frontRightSC;
	SpeedController* backLeftSC;
	SpeedController* backRightSC;
	RobotDrive* robotDrive;
	drivePIDOutput* headingPIDOutput;
	Gyro* gyro1;
	PIDController* headingPIDController;
	bool turning;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    float targetHeading;
	DriveMotors();
	void arcadeDrive(float dx, float dy, float dz);
	void InitDefaultCommand();
};

#endif
