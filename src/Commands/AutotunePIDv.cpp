#include "AutotunePIDv.h"
#include "Support/cPIDController.h"

AutotunePIDv::AutotunePIDv(Subsystem* sysin, double dCenter, double dDelta, double powerIn,
		cPIDController* cont[4], Encoder* enc[4], char nameIn[4][4])
{
	sys = sysin;
	center = dCenter;
	delta = dDelta;
	controller = cont;
	encoder = enc;
	target = center + delta;
	power = std::abs(powerIn);
	// Reset all encoders to zero, and move forward first
	for (int i = 0; i < 4; i++) {
		strcpy(name[i],nameIn[i]);
		strcat(name[i],".at");
		encoder[i]->Reset();
		controller[i]->SetMode(cPIDController::DIRECT);
		controller[i]->SetSetpoint(power);
	}
}

// Called just before this Command runs the first time
void AutotunePIDv::Initialize()
{
	for (int i = 0; i < 4; i++) {
		controller[i]->LogData(true,name[i]);
	}
}

// Called repeatedly when this Command is scheduled to run
void AutotunePIDv::Execute()
{
	// Check to see if it should be reversed.
	bool reverse = false;
	double distance;
	for (int i = 0; i < 4; i++) {
		distance = encoder[i]->GetDistance() - center;
		// Check against upper and lower deltas
		distance = std::max(distance - delta,0.0);
		distance = std::min(distance + delta,0.0);
		if (distance*target > 0.0) reverse = true;
	}
	// Reverse the target and add one to the count if reversed
	if (reverse) {
		count++;
		target = -target;
	}
	// Update all controllers
	for (int i = 0; i < 0; i++){
		controller[i]->SetSetpoint(target);
		controller[i]->UpdateController();
	}
	Wait(0.004);
}

// Make this return true when this Command no longer needs to run execute()
bool AutotunePIDv::IsFinished()
{
	return (count >= countMax);
}

// Called once after isFinished returns true
void AutotunePIDv::End()
{
	for (int i = 0; i < 4; i++) {
		controller[i]->LogData(false,name[i]);
		controller[i]->SetMode(cPIDController::OFF);
	}
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutotunePIDv::Interrupted()
{
	for (int i = 0; i < 4; i++) {
		controller[i]->LogData(false,name[i]);
		controller[i]->SetMode(cPIDController::OFF);
	}
}
