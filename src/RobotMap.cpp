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
#include "LiveWindow/LiveWindow.h"
// Test comment
// Connor's test comment


// Data for Robot Drive system
SpeedController* RobotMap::driveMotorsFrontLeftSC = NULL;
SpeedController* RobotMap::driveMotorsFrontRightSC = NULL;
SpeedController* RobotMap::driveMotorsBackLeftSC = NULL;
SpeedController* RobotMap::driveMotorsBackRightSC = NULL;
Encoder* RobotMap::driveMotorsFrontLeftEncoder = NULL;
Encoder* RobotMap::driveMotorsFrontRightEncoder = NULL;
Encoder* RobotMap::driveMotorsBackLeftEncoder = NULL;
Encoder* RobotMap::driveMotorsBackRightEncoder = NULL;
PIRateController* RobotMap::driveMotorsFrontLeftController = NULL;
PIRateController* RobotMap::driveMotorsFrontRightController = NULL;
PIRateController* RobotMap::driveMotorsBackLeftController = NULL;
PIRateController* RobotMap::driveMotorsBackRightController = NULL;
PIRateController* RobotMap::driveMotorsGyroController = NULL;
SingleMotor* RobotMap::driveMotorsFrontLeftMotor = NULL;
SingleMotor* RobotMap::driveMotorsFrontRightMotor = NULL;
SingleMotor* RobotMap::driveMotorsBackLeftMotor = NULL;
SingleMotor* RobotMap::driveMotorsBackRightMotor = NULL;
BuiltInAccelerometer* RobotMap::driveMotorsAccelerometer = NULL;
Gyro* RobotMap::driveMotorsGyro1 = NULL;

// Data for lift system
SpeedController* RobotMap::liftSC = NULL;
Encoder* RobotMap::liftEncoder = NULL;
PIRateController* RobotMap::liftRateController = NULL;
SingleMotor* RobotMap::liftMotor = NULL;
PIPositionController* RobotMap::liftPositionController = NULL;

void RobotMap::init() {
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
	LiveWindow* lw = LiveWindow::GetInstance();

	// Set Drive Motor Data
		//Speed Controllers
			driveMotorsFrontLeftSC = new Talon(1);
			driveMotorsFrontRightSC = new Talon(3);
			driveMotorsBackLeftSC = new Talon(2);
			driveMotorsBackRightSC = new Talon(0);

		// Add the gyro and accelerometer
			driveMotorsGyro1 = new Gyro(0);
			lw->AddSensor("Gyros", "Gyro", driveMotorsGyro1);
			driveMotorsGyro1->SetSensitivity(0.007);
			driveMotorsAccelerometer = new BuiltInAccelerometer();
			lw->AddSensor("Gyros", "Accelerometer", driveMotorsAccelerometer);

		// Add the Encoders.
			// Use # revolutions as the distance.
			driveMotorsBackRightEncoder = new Encoder(0, 1, false, Encoder::k4X);
			//Todo recalibrate
			driveMotorsBackRightEncoder->SetDistancePerPulse(0.0032);
			//
			driveMotorsFrontRightEncoder = new Encoder(6, 7, false, Encoder::k4X);
			driveMotorsFrontRightEncoder->SetDistancePerPulse(0.004332584);
			//
			driveMotorsFrontLeftEncoder = new Encoder(2, 3, true, Encoder::k4X);
			driveMotorsFrontLeftEncoder->SetDistancePerPulse(0.004655532);
			//
			driveMotorsBackLeftEncoder = new Encoder(4, 5, true, Encoder::k4X);
			driveMotorsBackLeftEncoder->SetDistancePerPulse(0.004019032);
		// Add the Controllers
			// Setup some parameters to be used by all controllers
			// ToDo: These need to be calibrated
			double pGain = 0.01;
			double iGain = 0.1;
            double maxOutput = 1.0; //Power setting
            double maxRate = 15.0; // rev/sec
            double controlSlope = 15.0;  // This is the change in rate per unit of power

			driveMotorsFrontLeftController = new PIRateController(pGain, iGain, maxOutput, maxRate, controlSlope);
			driveMotorsFrontRightController = new PIRateController(pGain, iGain, maxOutput, maxRate, controlSlope);
			driveMotorsBackLeftController = new PIRateController(pGain, iGain, maxOutput, maxRate, controlSlope);
			driveMotorsBackRightController = new PIRateController(pGain, iGain, maxOutput, maxRate, controlSlope);
			maxRate = 10.0; // deg/sec for gyro
			// ToDo: Calibrate the gyro settings
			pGain = 0.0;
			iGain = 0.0;
			controlSlope = 360.0;
			maxOutput = 0.3;
			driveMotorsGyroController = new PIRateController(pGain, iGain, maxOutput, maxRate, controlSlope);
		// Add the Motors
			driveMotorsFrontLeftMotor = new SingleMotor(driveMotorsFrontLeftSC, driveMotorsFrontLeftController, driveMotorsFrontLeftEncoder);
			driveMotorsFrontLeftMotor->scReversed = true;
			//
			driveMotorsFrontRightMotor = new SingleMotor(driveMotorsFrontRightSC, driveMotorsFrontRightController, driveMotorsFrontRightEncoder);
			driveMotorsFrontRightMotor->scReversed = false;
			//
			driveMotorsBackLeftMotor = new SingleMotor(driveMotorsBackLeftSC, driveMotorsBackLeftController, driveMotorsBackLeftEncoder);
			driveMotorsBackLeftMotor->scReversed = true;
			//
			driveMotorsBackRightMotor = new SingleMotor(driveMotorsBackRightSC, driveMotorsBackRightController, driveMotorsBackRightEncoder);
			driveMotorsBackRightMotor->scReversed = false;


		// Set up Live Windows
			lw->AddActuator("DriveMotors", "FrontLeftSC", (Talon*) driveMotorsFrontLeftSC);
			lw->AddSensor("DriveMotors", "FrontLeftEncoder", driveMotorsFrontLeftEncoder);
			lw->AddActuator("DriveMotors", "FrontRightSC", (Talon*) driveMotorsFrontRightSC);
			lw->AddSensor("DriveMotors", "FrontRightEncoder", driveMotorsFrontRightEncoder);
			lw->AddActuator("DriveMotors", "BackLeftSC", (Talon*) driveMotorsBackLeftSC);
			lw->AddSensor("DriveMotors", "BackLeftEncoder", driveMotorsBackLeftEncoder);
			lw->AddActuator("DriveMotors", "BackRightSC", (Talon*) driveMotorsBackRightSC);
			lw->AddSensor("DriveMotors", "BackRightEncoder", driveMotorsBackRightEncoder);

	// Set Lift Data
		// Speed Controller
			liftSC = new Victor(4);
			lw->AddActuator("Lift", "LiftMotor", (Victor*) liftSC);
		// The Encoder
			liftEncoder = new Encoder(8, 9, false, Encoder::k4X);
			lw->AddSensor("Lift", "LiftEncoder", liftEncoder);
			// Use inches for lift encoder
			liftEncoder->SetDistancePerPulse(0.0348);
		// The Controller
			// ToDo Reset controller parameters
			maxOutput = 0.5;
			maxRate = 10.0;
			pGain = 0.05;
			iGain = 0.1;
			controlSlope = 20.0;
			liftRateController = new PIRateController(pGain, iGain, maxOutput, maxRate, controlSlope);
		// The Motor
			liftMotor = new SingleMotor(liftSC, liftRateController, liftEncoder);
			liftMotor->scReversed = false;

		// The position controller
			// ToDo Tune these parameters, rewrite and use the position controller
			maxOutput = 0.5;
			maxRate = 10.0;
			pGain = 0.05;
			iGain = 0.1;
			controlSlope = 20.0;
			liftPositionController = new PIPositionController(pGain, iGain, maxOutput, liftEncoder);

}
