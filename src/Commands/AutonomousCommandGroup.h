#ifndef AutonomousCommandGroup_H
#define AutonomousCommandGroup_H

#include "Commands/CommandGroup.h"
#include "WPILib.h"

class AutonomousCommandGroup: public CommandGroup
{
private:
	int autoMode;
public:
	AutonomousCommandGroup(int autoModeIn);
};

#endif
