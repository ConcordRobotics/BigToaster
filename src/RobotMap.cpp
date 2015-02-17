// RobotBuilder Version: 1.5
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "RobotMap.h"
#include <iostream>
#include "LiveWindow/LiveWindow.h"

// Global data
float RobotMap::MotorWaitTime = 0.002; // 5ms

// Data for Robot Drive system
SpeedController *RobotMap::dmSCs[4] = {NULL, NULL, NULL, NULL};
cSpeedController *RobotMap::driveMotorsSCs[4] = {NULL, NULL, NULL, NULL};
Encoder *RobotMap::driveMotorsEncoders[4] = {NULL, NULL, NULL, NULL};
cPIDController* RobotMap::driveMotorsControllers[4] = {NULL, NULL, NULL, NULL};
PIDParams* RobotMap::driveMotorsRateGains = NULL;
ControllerLimits* RobotMap::driveMotorsLimits = NULL;
// ToDo Calibrate distance per rev
double RobotMap::distPerRev = 3.14159*8.0/12.0; // 8" wheels

// Ports for speed controllers
unsigned int RobotMap::driveMotorsPWMs[4] = {1,3,2,0};
unsigned int RobotMap:: driveMotorsPIOs[4][2] = { {2,3}, {6,7}, {4,5}, {0,1} };
bool RobotMap::driveMotorsSCReversed[4] = {true, false, true, false};
bool RobotMap::driveMotorsEncReversed[4] = {true, false, true, false};
char RobotMap::driveMotorsNames[4][4] = {"fl","fr","bl","br"};
float RobotMap::driveMotorsDPP[4] = {0.00419, 0.00433, 0.004019, 0.0028};

// Gyro
Gyro* RobotMap::gyro = NULL;
cPIDController* RobotMap::gyroController = NULL;
ControllerLimits* RobotMap::gyroLimits = NULL;
PIDParams* RobotMap::gyroRateGains = NULL;
PIDParams* RobotMap::gyroPositionGains = NULL;
cPIDOutput* RobotMap::gyroControllerOutput = NULL;

// Accelerometer
//BuiltInAccelerometer* RobotMap::driveMotorsAccelerometer = NULL;


// Data for lift system
SpeedController* RobotMap::liftSC = NULL;
Encoder* RobotMap::liftEncoder = NULL;
cPIDController* RobotMap::liftController = NULL;
ControllerLimits* RobotMap::liftLimits = NULL;
PIDParams* RobotMap::liftPositionGains = NULL;
PIDParams* RobotMap::liftRateGains = NULL;
DigitalInput* RobotMap::liftUpperSwitch = NULL;
DigitalInput* RobotMap::liftLowerSwitch = NULL;


// Data for claw
SpeedController* RobotMap::clawSC = NULL;
Encoder* RobotMap::clawEncoder = NULL;
cPIDController* RobotMap::clawController = NULL;
ControllerLimits* RobotMap::clawLimits = NULL;
PIDParams* RobotMap::clawPositionGains = NULL;
PIDParams* RobotMap::clawRateGains = NULL;





void RobotMap::init() {
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
	LiveWindow* lw = LiveWindow::GetInstance();


	std::cout << "Setting drive data\n";

	driveMotorsRateGains = new PIDParams(1.0, 0.0, 0.1, 1.0);
	// Set large position limits since there is no real limit
	driveMotorsLimits = new ControllerLimits(-1.0E30, 1.0E30, -15.0, 15.0, -1.0, 1.0);
	// Loop over motors to initialize Drive Motor data
	for (int i = 0; i < 4; i++) {
			dmSCs[i] = new Talon(driveMotorsPWMs[i]);
			driveMotorsSCs[i] = new cSpeedController (dmSCs[i],driveMotorsSCReversed[i]);
			driveMotorsEncoders[i] = new Encoder(driveMotorsPIOs[i][0],driveMotorsPIOs[i][1],
					driveMotorsEncReversed[i], Encoder::k4X);
			driveMotorsEncoders[i]->SetDistancePerPulse(driveMotorsDPP[i]);
			driveMotorsEncoders[i]->SetPIDSourceParameter(Encoder::kDistance);
			driveMotorsEncoders[i]->SetSamplesToAverage(127);
			driveMotorsControllers[i] = new cPIDController(driveMotorsRateGains, driveMotorsLimits,
					driveMotorsEncoders[i], driveMotorsSCs[i]);
			lw->AddActuator(driveMotorsNames[i], "SpeedController", (Talon*) dmSCs[i]);
			lw->AddSensor(driveMotorsNames[i], "Encoders", driveMotorsEncoders[i]);
	}
	std::cout << "Setting GyroData\n";

	// Add the gyro and accelerometer
		gyro = new Gyro(0);
		lw->AddSensor("Gyros", "Gyro", gyro);
		gyro->SetSensitivity(0.007);
		gyro->InitGyro();
		gyro->Reset();
		// No real limit for the gyros since angles wrap past 360 degrees
		// Should implement continuous mode for the controller
		gyroLimits = new ControllerLimits(-1.0E-30, 1.0E30, -10.0, 10.0, -1.0, 1.0);
		gyroRateGains = new PIDParams(0.02, 0.0, 0.0, 0.25);
		gyroPositionGains = new PIDParams(0.02, 1.0, 0.0, 0.25);
		gyroControllerOutput = new cPIDOutput();
		gyroController = new cPIDController(gyroRateGains, gyroLimits, gyro, gyroControllerOutput);

		// ToDo Add Accelerometer
		//driveMotorsAccelerometer = new BuiltInAccelerometer();
		//lw->AddSensor("Gyros", "Accelerometer", driveMotorsAccelerometer);


	std::cout << "Setting Lift Data\n";
	// Set Lift Data
		// Speed Controller
			liftSC = new Victor(4);
			lw->AddActuator("Lift", "LiftMotor", (Victor*) liftSC);
		// The Encoder
			liftEncoder = new Encoder(8, 9, false, Encoder::k4X);
			// Use inches for lift encoder
			liftEncoder->SetDistancePerPulse(0.0348);
			liftEncoder->SetPIDSourceParameter(Encoder::kDistance);
			lw->AddSensor("Lift", "LiftEncoder", liftEncoder);
		// The Controller
			liftLimits = new ControllerLimits(-1.0, 50.0, -10.0, 10.0, -1.0, 1.0);
			liftPositionGains = new PIDParams(0.0254, 1.1, 0.0, 0.1);
			liftRateGains = new PIDParams(0.0254, 1.1, 0.0, 0.1);
			liftController = new cPIDController(liftPositionGains, liftLimits, liftEncoder, liftSC);
			liftLowerSwitch = new DigitalInput(16);
			liftUpperSwitch = new DigitalInput(24);
			lw->AddSensor("Lift","UpperSwitch", liftUpperSwitch);
			lw->AddSensor("Lift","LowerSwitch", liftLowerSwitch);

			std::cout << "Setting Claw Data\n";
	 // Set Claw data
			// Speed Controller
				clawSC = new Victor(5);
				lw->AddActuator("Claw", "ClawMotor", (Victor*) clawSC);
				// The Encoder
				clawEncoder = new Encoder(14, 15, false, Encoder::k1X);
				lw->AddSensor("Claw", "ClawEncoder", clawEncoder);
				clawEncoder->SetDistancePerPulse(0.01612);
				clawEncoder->SetPIDSourceParameter(Encoder::kDistance);
				// The controller
				clawLimits = new ControllerLimits(-0.1, 1.1, -0.5, 0.5, -1.0, 1.0);
				clawPositionGains = new PIDParams(0.5, 0.2, 0.0, 1.0);
				clawRateGains = new PIDParams(0.5, 0.2, 0.0, 1.0);
				clawController = new cPIDController(clawPositionGains, clawLimits, clawEncoder, clawSC);

				// ToDo Try enabling derivative term on all controllers
				std::cout << "RobotMap complete\n";

}
