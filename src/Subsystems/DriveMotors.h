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

#include "Support/cPIDController.h"
#include "Support/cSpeedController.h"
#include "Support/cPIDOutput.h"

/**
 *
 *
 * @author ExampleAuthor
 */
class DriveMotors: public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
	double output[4];
	float headingTarget;
public:
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	cSpeedController* scs[4];
	cPIDController* controllers[4];
	Encoder *encoders[4];
	Gyro* gyro;
	cPIDOutput* gyroOutput;

	int gyroMode;
	cPIDController* headingCont;

	//BuiltInAccelerometer* accelerometer;
	DriveMotors();
	void ArcadeDrive(float dx, float dy, float dz);
	void InitDefaultCommand();
	void SetGyroMode(int modeIn);
	void SetHeadingTarget(float headingTargetIn);
	void Stop();
};

#endif
