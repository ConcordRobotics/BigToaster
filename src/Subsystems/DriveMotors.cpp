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

	flMotor = RobotMap::driveMotorsFrontLeftMotor;
	frMotor = RobotMap::driveMotorsFrontRightMotor;
	blMotor = RobotMap::driveMotorsBackLeftMotor;
	brMotor = RobotMap::driveMotorsBackRightMotor;

	gyro1 = RobotMap::driveMotorsGyro1;
	gyro1->Reset();
    gyroControlled = false;

	accelerometer = RobotMap::driveMotorsAccelerometer;

	headingCont = RobotMap::driveMotorsGyroController;

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
	if (gyroControlled) {
		headingCont->SetTarget(z);
		headingCont->SetPosition(gyro1->GetAngle());
		z = headingCont->controlOutput;
	}
    // Set up smart dashboard
	flMotor->OutputToDashboard("flMotor");
	frMotor->OutputToDashboard("frMotor");
	blMotor->OutputToDashboard("blMotor");
	brMotor->OutputToDashboard("brMotor");
	headingCont->OutputToDashboard("gyro");
    // Negate y for the joystick. Is this needed?
    y = -y;

    double wheelSpeeds[4];
    wheelSpeeds[0] = x + y + z;
    wheelSpeeds[1] = -x + y - z;
    wheelSpeeds[2] = -x + y + z;
    wheelSpeeds[3] = x + y - z;
    flMotor->SetTargetPower(wheelSpeeds[0]);
    frMotor->SetTargetPower(wheelSpeeds[1]);
    blMotor->SetTargetPower(wheelSpeeds[2]);
    brMotor->SetTargetPower(wheelSpeeds[3]);
    flMotor->UpdateRate();
    frMotor->UpdateRate();
    blMotor->UpdateRate();
    brMotor->UpdateRate();
    flMotor->SetPower();
    frMotor->SetPower();
    blMotor->SetPower();
    brMotor->SetPower();
}

void DriveMotors::Stop() {
	flMotor->sc->Set(0.0);
	frMotor->sc->Set(0.0);
	blMotor->sc->Set(0.0);
	brMotor->sc->Set(0.0);
}
