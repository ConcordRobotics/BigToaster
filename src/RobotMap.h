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

#define OUTPUT
#define DEBUG

#include "WPILib.h"
#include "Support/cEncoder.h"
#include "Support/cSpeedController.h"
#include "Support/cPIDController.h"
#include "Support/ControllerLimits.h"
#include "Support/cPIDOutput.h"


/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
class RobotMap {
public:
	// Some global constants
	static float MotorWaitTime;
	static Timer* timer;
	// Data for Drive Motors
	static enum {fl,fr,bl,br} motorPosition;
	// Declare all as arrays of pointers
	static cSpeedController *driveMotorsSCs[4];
	static SpeedController *dmSCs[4];
	static Encoder *driveMotorsEncoders[4];
	static cPIDController *driveMotorsControllers[4];
	static PIDParams* driveMotorsRateGains;
	static ControllerLimits* driveMotorsLimits;
	static unsigned int driveMotorsPWMs[4];
	static unsigned int driveMotorsPIOs[4][2];
	static bool driveMotorsSCReversed[4];
	static bool driveMotorsEncReversed[4];
	static float driveMotorsDPP[4];
	static char driveMotorsNames[4][4];
	static double distPerRev;

    // Gyro parameters
	static Gyro* gyro;
	static cPIDController* gyroController;
	static ControllerLimits* gyroLimits;
	static PIDParams* gyroRateGains;
	static PIDParams* gyroPositionGains;
	static cPIDOutput* gyroControllerOutput;

	// Accelerometer
	//static BuiltInAccelerometer* driveMotorsAccelerometer;

	// Data for Lift system
	static SpeedController* liftSC;
	static cSpeedController *liftCSC;
	static Encoder* liftEncoder;
	static cPIDController* liftController;
	static ControllerLimits* liftLimits;
 	static PIDParams* liftPositionGains;
 	static PIDParams* liftRateGains;
 	static DigitalInput* liftUpperSwitch;
 	static DigitalInput* liftLowerSwitch;

	// Data for the Claw Not enabled in telop yet
	static SpeedController* clawSC;
	static Encoder* clawEncoder;
	static cPIDController* clawController;
	static ControllerLimits* clawLimits;
 	static PIDParams* clawPositionGains;
 	static PIDParams* clawRateGains;

	static void init();
};
#endif
