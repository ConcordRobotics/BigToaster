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
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES


DriveMotors::DriveMotors() : Subsystem("DriveMotors") {

	for (int i=0; i < 4; i++) {
		scs[i] = RobotMap::driveMotorsSCs[i];
		encoders[i] = RobotMap::driveMotorsEncoders[i];
		controllers[i] = RobotMap::driveMotorsControllers[i];
		controllers[i]->SetMode(cPIDController::ENABLED);
		controllers[i]->SetSetpoint(0.0);
		scs[i]->Set(0.0);
		controllers[i]->LogData(true,RobotMap::driveMotorsNames[i]);
	}

	gyro1 = RobotMap::driveMotorsGyro1;
	gyro1->Reset();
    gyroControlled = false;

	accelerometer = RobotMap::driveMotorsAccelerometer;

	headingCont = RobotMap::driveMotorsGyroController;
	Stop();
}
    
void DriveMotors::InitDefaultCommand() {
	// Set the default command for a subsystem here.

	SetDefaultCommand(new DriveInTelop());

}

void DriveMotors::ArcadeDrive (float dx, float dy, float dz) {
	float x,y,z;
	x = dx;
	y = dy;
	z = dz;
	/*
	 if (gyroControlled) {
		headingCont->SetTarget(z);
		headingCont->SetPosition(gyro1->GetAngle());
		z = headingCont->controlOutput;
	}
	 */
    // Set up smart dashboard
	/*
	flMotor->OutputToDashboard("flMotor");
	frMotor->OutputToDashboard("frMotor");
	blMotor->OutputToDashboard("blMotor");
	brMotor->OutputToDashboard("brMotor");
	headingCont->OutputToDashboard("gyro"); */
    // Negate y for the joystick.
    y = -y;

    double wheelSpeeds[4];
    wheelSpeeds[0] = double(x + y + z);
    wheelSpeeds[1] = double(-x + y - z);
    wheelSpeeds[2] = double(-x + y + z);
    wheelSpeeds[3] = double(x + y - z);
    for (int i = 0; i < 4; i++) {
    	// ToDo hard code the 15.
    	controllers[i]->SetSetpoint(wheelSpeeds[i]*15);
    	controllers[i]->UpdateController(wheelSpeeds[i]);
    	controllers[i]->OutputToDashboard(RobotMap::driveMotorsNames[i]);
    	//RobotMap::driveMotorsSCs[i]->SafePWM::SetExpiration(1.0);
    }
    Wait(RobotMap::MotorWaitTime); // wait to avoid hogging CPU cycles
    //Stop();
}

void DriveMotors::Stop() {
	for (int i = 0; i < 4; i++) {
		scs[i]->Set(0.0);  // Could go back to inherited
	}
}
