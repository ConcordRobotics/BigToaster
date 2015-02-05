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

// Data for claw
SpeedController* RobotMap::clawSC = NULL;
Encoder* RobotMap::clawEncoder = NULL;
PIDController* RobotMap::clawPIDController = NULL;
float RobotMap::clawOpenPosition = 1.0;
float RobotMap::clawClosedPosition = 0.0;
float RobotMap::clawPositionGains[4] = {0.5, 0.01, 0.0, 0.0};
float RobotMap::clawRateGains[4] =  {0.5, 0.01, 0.0, 0.0};

void RobotMap::init() {
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
	LiveWindow* lw = LiveWindow::GetInstance();

	// Generic pointer to float
	float* p;

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
			//Todo DoubleCheck encoder distances for all wheels.
			// Turn wheels 10 times, and ensure that the distance on the
			// encoder is 10.  If not, adjust distance per pulse to
			// 10.0/distanceRead*currentValue
			driveMotorsBackRightEncoder->SetDistancePerPulse(0.0028);
			//
			driveMotorsFrontRightEncoder = new Encoder(6, 7, false, Encoder::k4X);
			driveMotorsFrontRightEncoder->SetDistancePerPulse(0.004332584);
			//
			driveMotorsFrontLeftEncoder = new Encoder(2, 3, true, Encoder::k4X);
			driveMotorsFrontLeftEncoder->SetDistancePerPulse(0.00532);
			//
			driveMotorsBackLeftEncoder = new Encoder(4, 5, true, Encoder::k4X);
			driveMotorsBackLeftEncoder->SetDistancePerPulse(0.004019032);
		// Add the Controllers
			// Setup some parameters to be used by all controllers
			double pGain = 0.01;
			double iGain = 0.01;
            double maxOutput = 1.0; //Power setting
            double maxRate = 15.0; // rev/sec
            double controlSlope = 15.0;  // This is the change in rate per unit of power

			driveMotorsFrontLeftController = new PIRateController(pGain, iGain, maxOutput, maxRate, controlSlope);
			driveMotorsFrontRightController = new PIRateController(pGain, iGain, maxOutput, maxRate, controlSlope);
			driveMotorsBackLeftController = new PIRateController(pGain, iGain, maxOutput, maxRate, controlSlope);
			driveMotorsBackRightController = new PIRateController(pGain, iGain, maxOutput, maxRate, controlSlope);
			maxRate = 10.0; // deg/sec for gyro
			// ToDo: Enable Gyro controller and calibrate
			// Probably best for Jim to do when in NH
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
			// ToDo Determine control slope for lift to improve controls
			// Run The lift up and down, and see what the encoder rate is vs. the controller output
			// Set the control slope as the rate/output.  We may need to have a different slope
			// going up vs. down.  For now, average the two.
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
			// ToDo Tune the lift position control and test that it works
			// Make sure that the rate controller above is working first.
			// Then in TelopMode,
			maxOutput = 0.5;
			maxRate = 10.0;
			pGain = 0.05;
			iGain = 0.1;
			controlSlope = 20.0;
			liftPositionController = new PIPositionController(pGain, iGain, maxOutput, liftEncoder);

	 // Set Claw data
			// Speed Controller
				clawSC = new Victor(5);
				lw->AddActuator("Claw", "ClawMotor", (Victor*) clawSC);
				// The Encoder
				clawEncoder = new Encoder(14, 15, false, Encoder::k4X);
				clawEncoder->SetSamplesToAverage(64);
				lw->AddSensor("Claw", "ClawEncoder", clawEncoder);
				// ToDo: Calibrate the claw encoder
				// Assume we want 0 to be fully closed, and 1 to be fully open.
				// In test mode, close the claw fully closed.  Note the encoder position.
				// Then fully open the claw, and note the position.  Subtract these two.
				// Set the distance per pulse to be 1.0/TotalDistance (since distance
				// per pulse is 1.0, the distance is actually the number of pulses.
				clawEncoder->SetDistancePerPulse(0.0161);
				clawEncoder->SetPIDSourceParameter(PIDSource::kDistance);
				// ToDo Set PID parameters for the Claw
				p = clawPositionGains;
				//p = clawRateGains;
				clawPIDController = new PIDController(p[0], p[1], p[2], p[3], clawEncoder, clawSC, 0.02);
				lw->AddActuator("Claw", "ClawPIDController", clawPIDController);
				clawPIDController->SetContinuous(false);
				clawPIDController->SetAbsoluteTolerance(0.01);
				// ToDo Set the max output range of the claw
				clawPIDController->SetOutputRange(-1.0, 1.0);
}
