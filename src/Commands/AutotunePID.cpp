#include "AutotunePID.h"
#include "Support/cPIDController.h"
#include "RobotMap.h"

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
	// Pick the direction
	double distance = encoder->GetDistance();
	if (distance < center) {
		target = center + delta;
		power = powerIn;
	} else {
		target = center - delta;
		power = -powerIn;
	}
	controller->SetMode(cPIDController::DIRECT);
	controller->SetSetpoint(power);
}

// Called just before this Command runs the first time
void AutotunePID::Initialize()
{
	controller->LogData(true,name);
	count = 0;
}

// Called repeatedly when this Command is scheduled to run
void AutotunePID::Execute()
{
	// Check to see if it should be reversed.
	double distance = encoder->GetDistance() - center;
	// Check against upper and lower deltas
	distance = std::max(distance - delta,0.0);
	distance = std::min(distance + delta,0.0);
	// if the  direction past the delta is in same direction
	// as the target, it's time to turn around
	if (distance*target > 0.0) {
		target = -target;
		count++;
	}
	controller->SetSetpoint(target);
	controller->UpdateController();
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
