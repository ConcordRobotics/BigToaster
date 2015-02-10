// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "Commands/LinearSysRate.h"
#include "Commands/LinearSysPosition.h"
#include "OI.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "SmartDashboard/SmartDashboard.h"
#include "Commands/AutonomousCommand.h"
#include "Commands/DriveInTelop.h"
#include "Commands/TestDrive.h"
#include "RobotMap.h"
#include "Commands/AutotunePID.h"
#include "Commands/AutotunePIDv.h"
#include "Support/cPIDController.h"
#include "RobotMap.h"

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

OI::OI() {
	// Process operator interface input here.
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
	//ToDo Adjust deadband and exp gains on joystick
	// The values below are for the x,y, and twist axes.
	// Set these based on driving "feel".
	// May be best to adjust when Jim in NH
	// We may want to add two modes the driver - fast vs. precise.
    float deadband[3] = {0.1, 0.1, 0.1};
    float eGain[3] = {0.25, 0.25, 0.0};
	joystick1 = new AdvancedJoystick(0, deadband, eGain);
	joystick2 = new Joystick(1);

	RaiseLiftRateButton = new JoystickButton(joystick2, 1);
	RaiseLiftRateButton->WhileHeld(new LinearSysRate(Robot::lift,Robot::lift, 10.0));
	//SmartDashboard::PutData("RaiseLiftRate", new LinearSysRate(Robot::lift,Robot::lift, 10.0));

	LowerLiftRateButton = new JoystickButton(joystick2, 2);
	LowerLiftRateButton->WhileHeld(new LinearSysRate(Robot::lift,Robot::lift, -10.0));
	//SmartDashboard::PutData("LowerLiftRate", new LinearSysRate(Robot::lift,Robot::lift,-10.0));

	LiftTopPosButton = new JoystickButton(joystick2, 4);
	LiftTopPosButton->WhenPressed(new LinearSysPosition(Robot::lift,Robot::lift,40.0));
	//SmartDashboard::PutData("LiftTopPos", new LinearSysPosition(Robot::lift,Robot::lift,30.0));

	LiftBottomPosButton = new JoystickButton(joystick2, 3);
	LiftBottomPosButton->WhenPressed(new LinearSysPosition(Robot::lift,Robot::lift,10.0));
	//SmartDashboard::PutData("LiftBottomPos", new LinearSysPosition(Robot::lift,Robot::lift,20.0));
//
	OpenClawRateButton = new JoystickButton(joystick2, 5);
	OpenClawRateButton->WhileHeld(new LinearSysRate(Robot::claw,Robot::claw, 0.2));
	//SmartDashboard::PutData("OpenClawRate", new LinearSysRate(Robot::claw,Robot::claw, 0.2));
//
	CloseClawRateButton = new JoystickButton(joystick2, 6);
	CloseClawRateButton->WhileHeld(new LinearSysRate(Robot::claw,Robot::claw,-0.2));
	//SmartDashboard::PutData("CloseClawRate", new LinearSysRate(Robot::claw,Robot::claw,-0.2));
//
	ClawOpenPosButton = new JoystickButton(joystick2, 7);
	// Tolerance should turn motor off when close enough
	ClawOpenPosButton->WhenPressed(new LinearSysPosition(Robot::claw,Robot::claw, 0.9, 0.03));
//	SmartDashboard::PutData("ClawOpenPos", new LinearSysPosition(Robot::claw,0.9));

	//This should apply some force on holding objects
	ClawClosedPosButton = new JoystickButton(joystick2, 8);
	ClawClosedPosButton->WhenPressed(new LinearSysPosition(Robot::claw,Robot::claw,0.0));
//	SmartDashboard::PutData("ClawClosedPos", new LinearSysPosition(Robot::claw,0.1));
	char name[] = "liftp.auto";
	//SmartDashboard::PutData("AutotuneLiftPos", new AutotunePID(Robot::lift, 30.0, 10.0, 15.0,
	//			RobotMap::liftPositionController, RobotMap::liftEncoder, name ));
//	strcpy(name,"liftr.auto");
//	SmartDashboard::PutData("AutotuneLiftRate", new AutotunePID(Robot::lift, 30.0, 10.0, 15.0,
//				RobotMap::liftRateController, RobotMap::liftEncoder, name ));
//	strcpy(name,"clawp.auto");
//	SmartDashboard::PutData("AutotuneClawPos", new AutotunePID(Robot::claw, 0.5,0.3, 0.5,
//				RobotMap::clawPositionController, RobotMap::clawEncoder, name ));
//	strcpy(name,"clawr.auto");
//	SmartDashboard::PutData("AutotuneClawRate", new AutotunePID(Robot::claw, 0.5,0.3, 0.5,
//				RobotMap::clawRateController, RobotMap::clawEncoder, name ));
	strcpy(name,"drive.auto");
	SmartDashboard::PutData("AutotuneDrive", new AutotunePIDv(Robot::driveMotors, 0.0,10.0, 10.0,
				RobotMap::driveMotorsControllers, RobotMap::driveMotorsEncoders, RobotMap::driveMotorsNames ));
	SmartDashboard::PutData("Autonomous Command", new AutonomousCommand());
	//SmartDashboard::PutData("DriveInTelop", new DriveInTelop());
    SmartDashboard::PutData("TestDrive", new TestDrive());


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
}

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

AdvancedJoystick* OI::getJoystick1() {
	return joystick1;
}

Joystick* OI::getJoystick2() {
	return joystick2;
}
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
