#include "AutotunePID.h"
#include "Support/cPIDController.h"
#include "RobotMap.h"
#include <iostream>

AutotunePID::AutotunePID(Subsystem* sysin, double dCenter, double dDelta, double powerIn,
		cPIDController* cont, Encoder* enc, char* nameIn)
{
	sys = sysin;
	center = dCenter;
	delta = dDelta;
	controller = cont;
	encoder = enc;
	name = new char[strlen(nameIn) + 1];
	name = strcpy(name,nameIn);
	power = powerIn;
	std::cout << "New Autotune " << nameIn << "\n";
}

// Called just before this Command runs the first time
void AutotunePID::Initialize()
{
	// Pick the direction
	double distance = encoder->GetDistance();
	if (distance < center) {
		power = std::abs(power);
	} else {
		power = -std::abs(power);
	}
	controller->SetMode(cPIDController::DIRECT);
	controller->SetSetpoint(power);
	controller->LogData(true,name);
	count = 0;
	controller->SetMode(cPIDController::DIRECT);
	std::cout << "Init Autotune " << name << "\n";
}

// Called repeatedly when this Command is scheduled to run
void AutotunePID::Execute()
{
	// Check to see if it should be reversed.
	double distance = encoder->GetDistance() - center;
	// Check against upper and lower deltas
	if (distance > 0) {
		distance = std::max(distance - delta,0.0);
	} else if  (distance < 0) {
		distance = std::min(distance + delta,0.0);
	}

	// if the  direction past the delta is in same direction
	// as the target, it's time to turn around
	std::cout << distance*power << "d*p ";
	if (distance*power > 0.0) {

		power = -power;
		count++;
	}
	std::cout << power << " power\n";
	controller->SetSetpoint(power);
	controller->UpdateController(0.3);

	Wait(RobotMap::MotorWaitTime);
}

// Make this return true when this Command no longer needs to run execute()
bool AutotunePID::IsFinished()
{
	return (count >= countMax);
}

// Called once after isFinished returns true
void AutotunePID::End()
{
	controller->LogData(false,name);
	controller->SetMode(cPIDController::OFF);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutotunePID::Interrupted()
{
	controller->LogData(false,name);
	controller->SetMode(cPIDController::OFF);
}
