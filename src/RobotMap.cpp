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
float RobotMap::MotorWaitTime = 0.001; // 5ms

// Data for Robot Drive system
SpeedController *RobotMap::dmSCs[4] = {NULL, NULL, NULL, NULL};
cSpeedController *RobotMap::driveMotorsSCs[4] = {NULL, NULL, NULL, NULL};
Encoder *RobotMap::driveMotorsEncoders[4] = {NULL, NULL, NULL, NULL};
cPIDController *RobotMap::driveMotorsControllers[4] = {NULL, NULL, NULL, NULL};
cPIDController* RobotMap::driveMotorsGyroController = NULL;
BuiltInAccelerometer* RobotMap::driveMotorsAccelerometer = NULL;
Gyro* RobotMap::driveMotorsGyro1 = NULL;
float RobotMap::driveMotorsGains[4] =  {0.1, 3.0, 0.0, 0.02};
float RobotMap::gyroRateGains[4] =  {1.0, 0.0, 0.0, 0.0};
// Encoder limits in revs/sec
float RobotMap::driveMotorEncoderLimits[2] = {-15.0,15.0};
float RobotMap::driveMotorSCLimits[2] = {-1.0,1.0};
// Ports for speed controllers
unsigned int RobotMap::driveMotorsPWMs[4] = {1,3,2,0};
unsigned int RobotMap:: driveMotorsPIOs[4][2] = { {2,3}, {6,7}, {4,5}, {0,1} };
bool RobotMap::driveMotorsSCReversed[4] = {true, false, true, false};
bool RobotMap::driveMotorsEncReversed[4] = {true, false, true, false};
char RobotMap::driveMotorsNames[4][4] = {"fl","fr","bl","br"};
float RobotMap::driveMotorsDPP[4] = {0.00419, 0.00433, 0.004019, 0.0028};

// Data for lift system
SpeedController* RobotMap::liftSC = NULL;
Encoder* RobotMap::liftEncoder = NULL;
cEncoder* RobotMap::liftPositionEncoder = NULL;
cPIDController* RobotMap::liftRateController = NULL;
cPIDController* RobotMap::liftPositionController = NULL;
float RobotMap::liftPositionGains[4] = {0.0254, 1.1, 0.275, 0.3}; // Feed forward term to counter gravity
float RobotMap::liftRateGains[4] =  {0.02, 1.1, 0.0, 0.3};
DigitalInput* RobotMap::liftUpperSwitch = NULL;
DigitalInput* RobotMap::liftLowerSwitch = NULL;


// Data for claw
SpeedController* RobotMap::clawSC = NULL;
Encoder* RobotMap::clawEncoder = NULL;
cEncoder* RobotMap::clawPositionEncoder = NULL;
cPIDController* RobotMap::clawPositionController = NULL;
cPIDController* RobotMap::clawRateController = NULL;
float RobotMap::clawOpenPosition = 1.0;
float RobotMap::clawClosedPosition = 0.0;
float RobotMap::clawPositionGains[4] = {1.0, 0.2, 0.0, 0.0};
float RobotMap::clawRateGains[4] =  {0.1, 0.2, 0.0, 1.0};

void RobotMap::init() {
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
	LiveWindow* lw = LiveWindow::GetInstance();

	// Generic pointer to float
	float* p;
	// Loop over motors to initialize Drive Motor data
	std::cout << "Setting drive data\n";
	for (int i = 0; i < 4; i++) {
			dmSCs[i] = new Talon(driveMotorsPWMs[i]);
			driveMotorsSCs[i] = new cSpeedController (dmSCs[i],driveMotorsSCReversed[i]);
			driveMotorsEncoders[i] = new Encoder(driveMotorsPIOs[i][0],driveMotorsPIOs[i][1],
					driveMotorsEncReversed[i], Encoder::k4X);
			driveMotorsEncoders[i]->SetDistancePerPulse(driveMotorsDPP[i]);
			driveMotorsEncoders[i]->SetPIDSourceParameter(Encoder::kRate);
			p = driveMotorsGains;
			driveMotorsControllers[i] = new cPIDController(p[0], p[1], p[2], p[3],
					driveMotorsEncoders[i], driveMotorsSCs[i]);
			driveMotorsControllers[i]->SetInputRange(-15.0,15.0);
			lw->AddActuator(driveMotorsNames[i], "SpeedController", (Talon*) dmSCs[i]);
			lw->AddSensor(driveMotorsNames[i], "Encoders", driveMotorsEncoders[i]);
			//char name[10];
			//strcpy(name,driveMotorsNames[i]);
			//driveMotorsControllers[i]->LogData(true,driveMotorsNames[i]);
	}

	std::cout << "Setting GyroData\n";
	//lw->AddActuator("DriveMotorSC", "FrontRight", (Talon*) driveMotorsSCs[1]);
	//lw->AddSensor("DriveMotors", "FrontRight", driveMotorsEncoders[1]);
	//lw->AddActuator("DriveMotorSC", "BackLeft", (Talon*) driveMotorsSCs[2]);
	//lw->AddSensor("DriveMotors", "BackLeft", driveMotorsEncoders[2]);
	//lw->AddActuator("DriveMotorSC", "BackRight", (Talon*) driveMotorsSCs[3]);
	//lw->AddSensor("DriveMotors", "BackRight", driveMotorsEncoders[3]);
	// Add the gyro and accelerometer
		driveMotorsGyro1 = new Gyro(0);
		lw->AddSensor("Gyros", "Gyro", driveMotorsGyro1);
		driveMotorsGyro1->SetSensitivity(0.007);
		//driveMotorsAccelerometer = new BuiltInAccelerometer();
		//lw->AddSensor("Gyros", "Accelerometer", driveMotorsAccelerometer);
		//p = gyroRateGains;
		//driveMotorsGyroController = new cPIDController(p[0], p[1], p[2], p[3], driveMotorsGyro1, );

	std::cout << "Setting Lift Data\n";
	// Set Lift Data
		// Speed Controller
			liftSC = new Victor(4);
			lw->AddActuator("Lift", "LiftMotor", (Victor*) liftSC);
		// The Encoder
			liftEncoder = new Encoder(8, 9, false, Encoder::k4X);
			lw->AddSensor("Lift", "LiftEncoder", liftEncoder);
			// Use inches for lift encoder
			liftEncoder->SetDistancePerPulse(0.0348);
			liftEncoder->SetPIDSourceParameter(Encoder::kRate);
		// The Controllers
			// ToDo Determine control slope for lift to improve controls
			// Run The lift up and down, and see what the encoder rate is vs. the controller output
			// Set the control slope as the rate/output.  We may need to have a different slope
			// going up vs. down.  For now, average the two.
			p = liftRateGains;
			liftRateController = new cPIDController(p[0], p[1], p[2], p[3],
					liftEncoder, liftSC);
			liftRateController->SetInputRange(-30.0,30.0);
			liftRateController->SetOutputRange(-1.0,1.0);

			liftPositionEncoder = new cEncoder(liftEncoder, cEncoder::kDistance );
			p = liftPositionGains;
			liftPositionController = new cPIDController(p[0], p[1], p[2], p[3],
					liftPositionEncoder, liftSC);
			liftPositionController->SetInputRange(0.0,50.0);
			liftPositionController->SetOutputRange(-1.0,1.0);
			liftLowerSwitch = new DigitalInput(18);
			liftUpperSwitch = new DigitalInput(19);
			lw->AddSensor("LiftSensors","UpperSwitch", liftUpperSwitch);
			lw->AddSensor("LiftSensors","LowerSwitch", liftLowerSwitch);
			std::cout << "Setting Claw Data\n";

	 // Set Claw data
			// Speed Controller
				clawSC = new Victor(5);
				lw->AddActuator("Claw", "ClawMotor", (Victor*) clawSC);
				// The Encoder
				clawEncoder = new Encoder(14, 15, false, Encoder::k1X);
				lw->AddSensor("Claw", "ClawEncoder", clawEncoder);
				// ToDo: Calibrate the claw encoder
				// Assume we want 0 to be fully closed, and 1 to be fully open.
				// In test mode, close the claw fully closed.  Note the encoder position.
				// Then fully open the claw, and note the position.  Subtract these two.
				// Set the distance per pulse to be 1.0/TotalDistance (since distance
				// per pulse is 1.0, the distance is actually the number of pulses.
				clawEncoder->SetDistancePerPulse(0.01612);
				clawEncoder->SetPIDSourceParameter(Encoder::kRate);

				// ToDo Set the max output range of the claw
				// Defaults should be ok for input/output ranges of claw

				p = clawRateGains;
				clawRateController = new cPIDController(p[0], p[1], p[2], p[3], clawEncoder, clawSC);
				clawRateController->SetInputRange(-1.0,1.0);
				clawRateController->SetOutputRange(-1.0,1.0);

				// ToDo Set PID parameters for the Claw
				// Add the distance based cEncoder
				clawPositionEncoder = new cEncoder(clawEncoder, cEncoder::kDistance );
				p = clawPositionGains;
				clawPositionController = new cPIDController(p[0], p[1], p[2], p[3],
						clawPositionEncoder, clawSC);
				clawPositionController->SetInputRange(0.0,1.0);
				clawPositionController->SetOutputRange(-1.0,1.0);
				std::cout << "RobotMap complete\n";
}
