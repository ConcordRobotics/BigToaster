// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "OI.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "SmartDashboard/SmartDashboard.h"
#include "Commands/AutonomousCommand.h"
#include "Commands/DriveInTelop.h"
#include "Commands/RaiseLift.h"
#include "Commands/LowerLift.h"
#include "Commands/TestDrive.h"
#include "Commands/LiftHigh.h"

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

OI::OI() {
	// Process operator interface input here.
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

	joystick1 = new Joystick(0);
	joystick2 = new Joystick(1);

	RaiseLiftButton = new JoystickButton(joystick2, 1);
	RaiseLiftButton->WhileHeld(new RaiseLift());
	SmartDashboard::PutData("RaiseLift", new RaiseLift());
	
	LowerLiftButton = new JoystickButton(joystick2, 2);
	LowerLiftButton->WhileHeld(new LowerLift());
	SmartDashboard::PutData("LowerLift", new LowerLift());

    // SmartDashboard Buttons
	SmartDashboard::PutData("Autonomous Command", new AutonomousCommand());
	SmartDashboard::PutData("DriveInTelop", new DriveInTelop());
    SmartDashboard::PutData("TestDrive", new TestDrive());
    SmartDashboard::PutData("LiftHigh", new LiftHigh());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
}

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

Joystick* OI::getJoystick1() {
	return joystick1;
}

Joystick* OI::getJoystick2() {
	return joystick2;
}
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
