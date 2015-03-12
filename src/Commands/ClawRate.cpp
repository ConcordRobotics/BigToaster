// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.
#include "Commands/ClawRate.h"
#include "Support/cPIDController.h"
ClawRate::ClawRate( float rateIn) {
	// Use requires() here to declare subsystem dependencies
	Requires(Robot::claw);
	sys = Robot::claw;
	rate = rateIn;
}

// Called just before this Command runs the first time
void ClawRate::Initialize() {
	sys->SetMode(cPIDController::DIRECT);
	// Reset the controller to target the current position
	sys->controller->Reset(sys->encoder->GetDistance());
	sys->SetSetpoint(rate);
	//std::cout << "COM START " << sys->name << " Rate " << RobotMap::timer->Get() << "\n";
}

// Called repeatedly when this Command is scheduled to run
void ClawRate::Execute() {
	sys->SetSetpoint(rate);
	sys->UpdateController();
}

// Make this return true when this Command no longer needs to run execute()
bool ClawRate::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void ClawRate::End() {
	sys->sc->Set(0.0);
	sys->Stop();
	//std::cout << "COM END " << sys->name << " Rate " << RobotMap::timer->Get() << "\n";
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ClawRate::Interrupted() {
	sys->sc->Set(0.0);
	sys->Stop();
	//std::cout << "COM INT " << sys->name << " Rate " << RobotMap::timer->Get() << "\n";
}
