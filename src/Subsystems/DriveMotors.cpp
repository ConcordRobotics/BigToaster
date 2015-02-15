// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.




#include "DriveMotors.h"
#include "../RobotMap.h"
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "../Commands/DriveInTelop.h"
#include <math.h>

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES


DriveMotors::DriveMotors() : Subsystem("DriveMotors") {

	for (int i=0; i < 4; i++) {
		scs[i] = RobotMap::driveMotorsSCs[i];
		encoders[i] = RobotMap::driveMotorsEncoders[i];
		controllers[i] = RobotMap::driveMotorsControllers[i];
		controllers[i]->SetMode(cPIDController::RATE);
		controllers[i]->SetRate(0.0);
		scs[i]->Set(0.0);
		controllers[i]->LogData(true,RobotMap::driveMotorsNames[i]);
		output[i] = 0.0;
	}

	gyro = RobotMap::gyro;
	gyro->Reset();
    gyroMode = cPIDController::DIRECT;
    SetGyroMode(cPIDController::DIRECT);
    gyroOutput = RobotMap::gyroControllerOutput;
	accelerometer = RobotMap::driveMotorsAccelerometer;

	headingCont = RobotMap::gyroController;
	Stop();
}

void DriveMotors::SetGyroMode(int modeIn) {
	gyroMode = modeIn;
	headingCont->SetMode(gyroMode);
	if (gyroMode == cPIDController::RATE) {
		headingCont->SetPIDParams(RobotMap::gyroRateGains);
	} else if (gyroMode == cPIDController::POSITION) {
		headingCont->SetPIDParams(RobotMap::gyroPositionGains);
	}
}
void DriveMotors::InitDefaultCommand() {
	// Set the default command for a subsystem here.

	SetDefaultCommand(new DriveInTelop(cPIDController::DIRECT));

}

void DriveMotors::SetHeadingTarget(float headingTargetIn ) {
	headingTarget = headingTargetIn;
}

void DriveMotors::ArcadeDrive (float dx, float dy, float dz) {
	float x,y,z;
	x = dx;
	y = dy;
	z = dz;
	// Negate the y-axis for joystick
    y = -y;

	 if (gyroMode == cPIDController::RATE) {
		 headingCont->SetRate(z*RobotMap::gyroLimits->rMax);
		 headingCont->UpdateController(gyroOutput->Get());
	 } else if (gyroMode == cPIDController::POSITION) {
		 float curHeading = gyro->GetAngle();
		 // Calculate the closes direction to get to the desired angle since the angles wrap 360 degrees
		 float delHeading = headingTarget - curHeading;
		 // Get the change in heading to lie in range 0 to 360
		 delHeading = delHeading - 360.0*floor(delHeading/360.0);
		 // Transform deltaHeading to go from -180 to +180 to get closest path
		 // May need to tweak this to knock it off 180
		 if (delHeading > 180.0) delHeading = delHeading - 360.0;
		 headingCont->SetSetpoint(curHeading + delHeading);
		 headingCont->UpdateController(gyroOutput->Get());
	 } else {
		 headingCont->SetSetpoint(z);
		 headingCont->UpdateController(0.0);
	 }
	 // Get the output from the controller
	 z = gyroOutput->Get();
	 headingCont->OutputToDashboard("gyro");



    double wheelSpeeds[4];
    wheelSpeeds[0] = double(x + y + z);
    wheelSpeeds[1] = double(-x + y - z);
    wheelSpeeds[2] = double(-x + y + z);
    wheelSpeeds[3] = double(x + y - z);
    for (int i = 0; i < 4; i++) {
    	// ToDo hard code the 15.
    	controllers[i]->SetRate(wheelSpeeds[i]*15);
    	output[i]=controllers[i]->UpdateController(output[i]);
    	controllers[i]->OutputToDashboard(RobotMap::driveMotorsNames[i]);
    	//RobotMap::driveMotorsSCs[i]->SafePWM::SetExpiration(1.0);
    }
    Wait(RobotMap::MotorWaitTime); // wait 5ms to avoid hogging CPU cycles
    //Stop();
}

void DriveMotors::Stop() {
	for (int i = 0; i < 4; i++) {
		scs[i]->Set(0.0);  // Could go back to inherited
	}
}
