// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.
#include "Commands/LinearHoldPosition.h"

LinearHoldPosition::LinearHoldPosition(Subsystem* sysIn, LinearSystem* linSys)
                  : LinearSysPosition(sysIn, linSys, 0.0, -1.0) {
}


// Called just before this Command runs the first time
void LinearHoldPosition::Initialize() {
	sys->SetMode(cPIDController::POSITION);
	position = sys->encoder->GetDistance();
	sys->controller->Reset(position);
	sys->SetSetpoint(position);
	std::cout << "COM START " << sys->name << " Hold " << RobotMap::timer->Get() << "\n";
}
