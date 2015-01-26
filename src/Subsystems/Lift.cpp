// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Lift.h"
#include "../RobotMap.h"
#include "SmartDashboard/SmartDashboard.h"
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

Lift::Lift() : Subsystem("Lift") {
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	liftSC = RobotMap::liftSC;
    liftMotor = RobotMap::liftMotor;
    liftEncoder = RobotMap::liftEncoder;
    liftEncoder->Reset();
    targetPosition = 0.0;
	liftMotor->OutputToDashboard("Lift");
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
}
    
void Lift::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	//SetDefaultCommand(new MySpecialCommand());
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
	//SetDefaultCommand(NULL);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
}

void Lift::SetPower(float power){
	liftMotor->OutputToDashboard("Lift");
	liftMotor->SetTargetPower(power);
	liftMotor->UpdateRate();
	liftMotor->SetPower();

}


// Put methods for controlling this subsystem
// here. Call these from Commands.

