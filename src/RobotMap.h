// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#ifndef ROBOTMAP_H
#define ROBOTMAP_H

#include "WPILib.h"
#include "Support/cEncoder.h"
#include "Support/cSpeedController.h"
#include "Support/cPIDController.h"


/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
class RobotMap {
public:

	// Data for Drive Motors
	static enum {fl,fr,bl,br} motorPosition;
	// Declare all as arrays of pointers
	static cSpeedController *driveMotorsSCs[4];
	static SpeedController *dmSCs[4];
	static Encoder *driveMotorsEncoders[4];
	static cPIDController *driveMotorsControllers[4];
	static cPIDController* driveMotorsGyroController;
	static Gyro* driveMotorsGyro1;
	static BuiltInAccelerometer* driveMotorsAccelerometer;
	static float driveMotorsGains[4];
	static float gyroRateGains[4];
	static float driveMotorEncoderLimits[2];
	static float driveMotorSCLimits[2];
	static unsigned int driveMotorsPWMs[4];
	static unsigned int driveMotorsPIOs[4][2];
	static bool driveMotorsSCReversed[4];
	static bool driveMotorsEncReversed[4];
	static float driveMotorsDPP[4];
	static char driveMotorsNames[4][4];

	// Data for Lift system
	static SpeedController* liftSC;
	static Encoder* liftEncoder;
	static cEncoder* liftPositionEncoder;
	static cPIDController* liftRateController;
    static cPIDController* liftPositionController;
	static void init();
 	static float liftPositionGains[4];
 	static float liftRateGains[4];

	// Data for the Claw Not enabled in telop yet
	static SpeedController* clawSC;
	static Encoder* clawEncoder;
 	static cPIDController* clawPositionController;
 	static cPIDController* clawRateController;
 	static float clawOpenPosition;
 	static float clawClosedPosition;
 	static float clawPositionGains[4];
 	static float clawRateGains[4];
};
#endif
