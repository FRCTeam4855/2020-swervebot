/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// FRC TEAM 4855 ROBOT CODE
// 2020 GAME INFINITE RECHARGE

// Swerve-bot code: Robot

package frc.robot;
// package edu.christmas.2012;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.christmas.2012.ben10watch.used;
// import com.nintendo.gameboy.pikachu;

public class Robot extends TimedRobot {
			
	// CONTROL CONSTANTS
	//=======================================
	final double CONTROL_SPEEDREDUCTION = .6; 	  			  	// teleop drivetrain inputs are multiplied by this number when turbo is NOT engaged
	final double CONTROL_SPEEDREDUCTION_PRECISION = 3.2;		// teleop drivetrain inputs are divided by this number when precision trigger is engaged
	final double CONTROL_DEADZONE = 0.23;       			    // minimum value before joystick inputs will be considered on the swerves
	final boolean CONTROL_AUTOFAULT_HANDLE = false;				// whether or not the robot will automatically react to a faulty wheel and flip to tank drive
	boolean INTERFACE_SINGLEDRIVER = false;  		  			// whether or not to enable or disable single driver input (press START to switch between controllers)
	
	// OTHER CONSTANTS
	final static double ROBOT_WIDTH = 29;
	final static double ROBOT_LENGTH = 29;
	final static double ROBOT_R = Math.sqrt(Math.pow(ROBOT_LENGTH, 2) + Math.pow(ROBOT_WIDTH, 2));
	final static double ENC_TO_DEG = 4.588888;		// formerly 1.158333
	final static double ABS_TO_DEG = 11.244444;
	final static double ENC_360 = 1652;				// formerly 417 with original Spark-driven swerves
	final static double IN_TO_ENC = 10.394;

	// BEGINNING VARIABLES
	int wheelTune = 0; 							// Remembers what wheel we are tweaking in test mode
	int singleDriverController = 0; 			// port number of controller to operate
	boolean emergencyTank = false; 				// true if the robot is in emergency tank drive mode
	boolean reverseRotate = false;				// can probably get deleted
	boolean driverOriented = true;				// true = driver oriented, false = robot oriented
	static double matchTime = 0;				// the calculated match time from the driver station
	boolean emergencyReadjust = false;			// if hell has come to earth and you need to manually adjust wheels during a match, this will be enabled
	String playType = "MATCH";					// whether to act as if in a match ("MATCH") or testing ("TEST")
	// All for calculating wheel speed/angle, if you need to read from a motor don't pull from these
	static double a, b, c, d, max, temp, rads; 
	static double encoderSetpointA, encoderSetpointB, encoderSetpointC, encoderSetpointD;
	static double jStr, jFwd, jRcw;
	static double wheelSpeed1, wheelSpeed2, wheelSpeed3, wheelSpeed4;
	static double overrideSTR = 0, overrideFWD = 0, overrideRCW = 0;		// set these variables to override drive() values
	// Gradual starts/stops in teleop
	static double wheelSpeedActual1 = 0, wheelSpeedActual2 = 0, wheelSpeedActual3 = 0, wheelSpeedActual4 = 0;
	static Timer wheelSpeedTimer = new Timer();

	// AUTONOMOUS VARIABLES
	int a_startType = 4;				// correlates to the driver station you are standing behind
	char a_autoType = 'a';				// correlates to the routine type
	boolean a_truncateRoutine = false;	// truncates the routine to a predefined step
  	//=======================================
  
	// DEFINING HARDWARE
	//=======================================
	// Define swerve wheel classes
	static Wheel wheel[] = {
		new Wheel(new TalonSRX(1), new CANSparkMax(6, MotorType.kBrushless), new AnalogInput(2), 2),// front right
		new Wheel(new TalonSRX(2), new CANSparkMax(7, MotorType.kBrushless), new AnalogInput(3), 3),// front left
		new Wheel(new TalonSRX(3), new CANSparkMax(8, MotorType.kBrushless), new AnalogInput(1), 0),// back left
		new Wheel(new TalonSRX(4), new CANSparkMax(9, MotorType.kBrushless), new AnalogInput(0), 1)// back right	
	};
	
	// Xbox controllers
	Joystick controlDriver = new Joystick(0);			// the joystick responsible for driving
	Joystick controlOperator = new Joystick(1);			// the joystick responsible for operator controls
	static Joystick controlWorking;  					// the controller currently being read from, usually used just for one-driver control

	// NavX Constructor
	public static AHRS gyro = new AHRS(SPI.Port.kMXP);

	// Blinkin Constructor
	Blinkin leds = new Blinkin(10);

	// Lidar Constructor
	static Lidar lidar = new Lidar(3);

	// Limelight Constructor
	static Limelight limelight = new Limelight();

	// Shooter Constructor
	static Shooter shooter = new Shooter(10, 0, 5);

	// Intake Constructor
	static Intake intake = new Intake(2, 1, 0, 1, 2);

	// Climber Constructor
	PWMVictorSPX climber = new PWMVictorSPX(3);

	// USB Camera Constructor
	UsbCamera camera;
	//=======================================
	
	// COMPUTATIONAL SOFTWARE STUFF
	//=======================================
	// Action queues
	private ActionQueueHandler aqHandler;
	private ActionQueue[] aqArray = new ActionQueue[] {
		new ActionQueue(true),	// QUEUE_ANGLE
		new ActionQueue(true),	// QUEUE_DRIVESTRAIGHT
		new ActionQueue(false),	// QUEUE_SHOOTVOLLEY
		new ActionQueue(true),	// QUEUE_LIMELIGHTANGLE

		new ActionQueue(true),	// QUEUE_AUTONOMOUS_1A
		new ActionQueue(true),	// QUEUE_AUTONOMOUS_1B
		new ActionQueue(true),	// QUEUE_AUTONOMOUS_2A
		new ActionQueue(true),	// QUEUE_AUTONOMOUS_2B
		new ActionQueue(true),	// QUEUE_AUTONOMOUS_2C
		new ActionQueue(true),	// QUEUE_AUTONOMOUS_3A
		new ActionQueue(true),	// QUEUE_AUTONOMOUS_4A
		new ActionQueue(true),	// MADDEN_SPINMOVE
	};

	// Reference IDs for action queues
	final int QUEUE_ANGLE = 0;
	final int QUEUE_DRIVESTRAIGHT = 1;
	final int QUEUE_SHOOTVOLLEY = 2;
	final int QUEUE_LIMELIGHTANGLE = 3;
	final int QUEUE_AUTONOMOUS_1A = 4;
	final int QUEUE_AUTONOMOUS_1B = 5;
	final int QUEUE_AUTONOMOUS_2A = 6;
	final int QUEUE_AUTONOMOUS_2B = 7;
	final int QUEUE_AUTONOMOUS_2C = 8;
	final int QUEUE_AUTONOMOUS_3A = 9;
	final int QUEUE_AUTONOMOUS_4A = 10;
	final int MADDEN_SPINMOVE = 21;
	//=======================================

	// End of variable definitions

	/**
	 * Drives the robot and invokes the Wheel method process(). If emergency tank mode is enabled, then the swerve wheels will behave as two pairs of tank wheels.
	 */
	public void drive() {
		if (!emergencyTank) {
			if (!emergencyReadjust && (!controlWorking.getRawButton(Utility.BUTTON_LSTICK) && !controlWorking.getRawButton(Utility.BUTTON_RSTICK))) {
				// Drive the robot, will adjust driverOriented based on toggled input
				jFwd = -controlWorking.getRawAxis(1);if (Math.abs(jFwd) < CONTROL_DEADZONE) jFwd = 0;
				if (!controlWorking.getRawButton(Utility.BUTTON_RB) && controlWorking.getRawAxis(2) < .7) jFwd *= CONTROL_SPEEDREDUCTION;
				if (controlWorking.getRawAxis(2) >= .7) jFwd /= CONTROL_SPEEDREDUCTION_PRECISION;

				jStr = controlWorking.getRawAxis(0);if (Math.abs(jStr) < CONTROL_DEADZONE) jStr = 0;
				if (!controlWorking.getRawButton(Utility.BUTTON_RB) && controlWorking.getRawAxis(2) < .7) jStr *= CONTROL_SPEEDREDUCTION;
				if (controlWorking.getRawAxis(2) >= .7) jStr /= CONTROL_SPEEDREDUCTION_PRECISION;

				jRcw = controlWorking.getRawAxis(4);if (Math.abs(jRcw) < CONTROL_DEADZONE) jRcw = 0;
				if (!controlWorking.getRawButton(Utility.BUTTON_RB) && controlWorking.getRawAxis(2) < .7) jRcw *= CONTROL_SPEEDREDUCTION;
				if (controlWorking.getRawAxis(2) >= .7) jRcw /= CONTROL_SPEEDREDUCTION_PRECISION;

				if (reverseRotate) jRcw = -jRcw;
				boolean turnWheels;					// whether or not the wheels should attempt to spin to a new angle
				if ((jFwd != 0 && jStr != 0 && jRcw != 0)) turnWheels = false; else turnWheels = true;
				
				// Pull from override information
				if (overrideFWD != 0) {
					jFwd = overrideFWD;
					turnWheels = true;
				}
				if (overrideSTR != 0) {
					jStr = overrideSTR;
					turnWheels = true;
				}
				if (overrideRCW != 0) {
					jRcw = overrideRCW;
					turnWheels = true;
				}
				
				if (turnWheels && !controlDriver.getRawButton(Utility.BUTTON_X)) swerve(jFwd, jStr, jRcw, driverOriented);
			}
		} else {
			// Emergency tank drive
			Utility.setAllPIDSetpoints(0, wheel);
			Utility.resetAllWheels(wheel);
			wheel[0].motorDrive.set(controlWorking.getRawAxis(5));
			wheel[3].motorDrive.set(controlWorking.getRawAxis(5));
			wheel[2].motorDrive.set(controlWorking.getRawAxis(1));
			wheel[1].motorDrive.set(controlWorking.getRawAxis(1));
		}

		for (Wheel w : wheel) {
			w.process();
		}
	}

	/**
	 * This adjusts the angle of the wheels and sets their speed based on joystick/autonomous input.
	 * 
	 * @param FWD The desired forward speed of the robot
	 * @param STR The desired strafing speed of the robot
	 * @param RCW The desired rotation speed of the robot
	 */
	public static void swerve(double FWD, double STR, double RCW, boolean driverOriented) {
		if (driverOriented) {
			rads = gyro.getYaw() * Math.PI / 180;
			temp = FWD * Math.cos(rads) + STR * Math.sin(rads);
			STR = -FWD * Math.sin(rads) + STR * Math.cos(rads);
			FWD = temp;
		}

		a = STR - RCW * (ROBOT_LENGTH / ROBOT_R);
		b = STR + RCW * (ROBOT_LENGTH / ROBOT_R);
		c = FWD - RCW * (ROBOT_WIDTH / ROBOT_R);
		d = FWD + RCW * (ROBOT_WIDTH / ROBOT_R);

		//1..4: front_right, front_left, rear_left, rear_right

		wheelSpeed1 = Math.sqrt(Math.pow(b, 2) + Math.pow(c, 2));
		wheelSpeed2 = Math.sqrt(Math.pow(b, 2) + Math.pow(d, 2));
		wheelSpeed3 = Math.sqrt(Math.pow(a, 2) + Math.pow(d, 2));
		wheelSpeed4 = Math.sqrt(Math.pow(a, 2) + Math.pow(c, 2));

		encoderSetpointA = wheel[0].calculateWheelAngle(b, c);
		wheel[0].setSetpoint((int) encoderSetpointA);
		
		encoderSetpointB = wheel[1].calculateWheelAngle(b, d);
		wheel[1].setSetpoint((int) encoderSetpointB);
		
		encoderSetpointC = wheel[2].calculateWheelAngle(a, d);
		wheel[2].setSetpoint((int) encoderSetpointC);
		
		encoderSetpointD = wheel[3].calculateWheelAngle(a, c);
		wheel[3].setSetpoint((int) encoderSetpointD);

		// If a wheel calculated itself to a value above 1, reduce all wheel speeds (?? idk what this really does)
		max = Math.max(wheelSpeed1, Math.max(wheelSpeed2, Math.max(wheelSpeed3, wheelSpeed4)));
		if (max > 1) {wheelSpeed1 /= max; wheelSpeed2 /= max; wheelSpeed3 /= max; wheelSpeed4 /= max;}
		
		wheelSpeed1 *= wheel[0].getFlip();
		wheelSpeed2 *= wheel[1].getFlip();
		wheelSpeed3 *= wheel[2].getFlip();
		wheelSpeed4 *= wheel[3].getFlip();
		
		// Tween wheel speeds
		if (wheelSpeedTimer.get() > 0.1) {
			if (wheelSpeed1 - wheelSpeedActual1 > 0.1) {wheelSpeedActual1 += 0.1;} else if (wheelSpeed1 - wheelSpeedActual1 < -0.1) {wheelSpeedActual1 -= 0.1;} else {wheelSpeedActual1 = wheelSpeed1;}
			if (wheelSpeed2 - wheelSpeedActual2 > 0.1) {wheelSpeedActual2 += 0.1;} else if (wheelSpeed2 - wheelSpeedActual2 < -0.1) {wheelSpeedActual2 -= 0.1;} else {wheelSpeedActual2 = wheelSpeed2;}
			if (wheelSpeed3 - wheelSpeedActual3 > 0.1) {wheelSpeedActual3 += 0.1;} else if (wheelSpeed3 - wheelSpeedActual3 < -0.1) {wheelSpeedActual3 -= 0.1;} else {wheelSpeedActual3 = wheelSpeed3;}
			if (wheelSpeed4 - wheelSpeedActual4 > 0.1) {wheelSpeedActual4 += 0.1;} else if (wheelSpeed4 - wheelSpeedActual4 < -0.1) {wheelSpeedActual4 -= 0.1;} else {wheelSpeedActual4 = wheelSpeed4;}
			wheelSpeedTimer.reset();
		}
		
		wheel[0].motorDrive.set(wheelSpeedActual1);
		wheel[1].motorDrive.set(wheelSpeedActual2);
		wheel[2].motorDrive.set(wheelSpeedActual3);
		wheel[3].motorDrive.set(wheelSpeedActual4);
	}

	/**
	 * Readjust the wheels manually
	 */
	public void readjust() {
		if (controlDriver.getRawButton(1)) wheelTune = 0;
		if (controlDriver.getRawButton(2)) wheelTune = 1;
		if (controlDriver.getRawButton(3)) wheelTune = 2;
		if (controlDriver.getRawButton(4)) wheelTune = 3;
		
		// Adjust wheel angle
		if (controlDriver.getRawButton(5)) wheel[wheelTune].motorAngle.set(ControlMode.PercentOutput, 0.3);
		else if (controlDriver.getRawButton(6)) wheel[wheelTune].motorAngle.set(ControlMode.PercentOutput, -0.3); else wheel[wheelTune].motorAngle.set(ControlMode.PercentOutput, 0);
		
		// Spin wheels
		if (controlDriver.getRawAxis(2) > .09) wheel[wheelTune].motorDrive.set(controlDriver.getRawAxis(2) / 2);
		else if (controlDriver.getRawAxis(3) > .09) wheel[wheelTune].motorDrive.set(-controlDriver.getRawAxis(3) / 2);
		else wheel[wheelTune].motorDrive.set(0);
	}

	// <--- ROBOT INITIALIZATION --->
	
	/**
	 * This function is called when the robot is turned on
	 */
	@Override
	public void robotInit() {

		// Configure USB camera
		camera = CameraServer.getInstance().startAutomaticCapture(0);
		camera.setBrightness(20);
		camera.setExposureManual(50);
		camera.setResolution(160, 120);
		camera.setFPS(17);
		
		aqHandler = new ActionQueueHandler(aqArray);

		// Feed action queues, they hunger for your command

		aqHandler.getQueue(QUEUE_ANGLE).queueFeed(ActionQueue.Command.TURN_TO_ANGLE, 1, 80, false, 0, 0, 0);

		aqHandler.getQueue(QUEUE_DRIVESTRAIGHT).queueFeed(ActionQueue.Command.DRIVE_STRAIGHT, 1, 120, false, .3, 0, 0);

		aqHandler.getQueue(QUEUE_SHOOTVOLLEY).queueFeed(ActionQueue.Command.RUN_FLYWHEEL, 0, 2.7, true, 3320, 0, 0);
		aqHandler.getQueue(QUEUE_SHOOTVOLLEY).queueFeed(ActionQueue.Command.WAIT_FOR_SENSOR, 0, 0.1, false, 1, 0, 0);
		aqHandler.getQueue(QUEUE_SHOOTVOLLEY).queueFeed(ActionQueue.Command.FEED_BALL, 0.1, 2.4, true, 0, 0, 0);

		aqHandler.getQueue(QUEUE_LIMELIGHTANGLE).queueFeed(ActionQueue.Command.ANGLE_TO_LIMELIGHT_X, 0, 100, false, 0, 0, 0);

		// Autonomous Routine 1A - Start in front of station 1, shoot, pick up balls directly behind machine and shoot again
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1A).queueFeed(ActionQueue.Command.ANGLE_TO_LIMELIGHT_X, 0, 150, false, 0, 0, 0);	// find target from Limelight
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1A).queueFeed(ActionQueue.Command.RUN_FLYWHEEL, 0, 350, true, 0, 1, 0);				// run shooter based on lidar input
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1A).queueFeed(ActionQueue.Command.FEED_BALL, 120, 240, true, 0, 0, 0);				// feed balls into into shooter
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1A).queueFeed(ActionQueue.Command.TURN_TO_ANGLE, 340, 460, false, 180, 0, 0);		// turn the robot around
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1A).queueFeed(ActionQueue.Command.DRIVE_STRAIGHT, 400, 550, true, -.3, 0, 0);		// approach other balls
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1A).queueFeed(ActionQueue.Command.RUN_INTAKE_WHEELS, 440, 550, true, .7, 0, 0);		// suck in balls
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1A).queueFeed(ActionQueue.Command.TURN_TO_ANGLE, 580, 660, false, 0, 0, 0);			// turn back around to 0 degrees
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1A).queueFeed(ActionQueue.Command.DRIVE_STRAIGHT, 580, 690, false, .3, 0, 0);		// drive toward goal
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1A).queueFeed(ActionQueue.Command.RUN_FLYWHEEL, 700, 900, true, 0, 1, 0);			// run shooter based on lidar input
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1A).queueFeed(ActionQueue.Command.FEED_BALL, 703, 830, true, 0, 0, 0);				// feed balls into shooter

		// Autonomous Routine 4A - Autocross from any location
		aqHandler.getQueue(QUEUE_AUTONOMOUS_4A).queueFeed(ActionQueue.Command.DRIVE_STRAIGHT, 90, 150, false, .3, 0, 0);

		// Put default auto chooser values to the dashboard
		SmartDashboard.putNumber("Auto: Station #", a_startType);
		SmartDashboard.putString("Auto: Routine Type", String.valueOf(a_autoType));
		SmartDashboard.putBoolean("Auto: Truncate", a_truncateRoutine);
	}
	
	/**
	 * This function is called immediately when the robot is disabled
	 */
	@Override
	public void disabledInit() {
		Utility.cleanSlateAllWheels(wheel);
		Utility.powerAllWheels(false, wheel);	
		Utility.setAllPIDSetpoints(0, wheel);
		shooter.resetPivotPosition();
		shooter.killFlywheel();
		shooter.killFeeder();
		aqHandler.killQueues();
		intake.stop();
	}

	@Override
	public void disabledPeriodic() {
		// Display autonomous controls
		SmartDashboard.putNumber("Auto: Station #", a_startType);
		SmartDashboard.putString("Auto: Routine Type", String.valueOf(a_autoType));
		SmartDashboard.putBoolean("Auto: Truncate", a_truncateRoutine);

		a_startType = (int) SmartDashboard.getNumber("Auto: Station #", 4);
		a_autoType = SmartDashboard.getString("Auto: Routine Type", "a").charAt(0);
		a_truncateRoutine = SmartDashboard.getBoolean("Auto: Truncate", a_truncateRoutine);
	}
	
	/**
	 * This function is called immediately when autonomous begins
	 */
	@Override
	public void autonomousInit() {
		wheelSpeedTimer.start();
		wheelSpeedTimer.reset();
		gyro.reset();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		// haha it's empty
	}
	
	/**
	 * This function is called when teleop begins
	 */
	public void teleopInit() {
		Utility.cleanSlateAllWheels(wheel);
		wheelSpeedTimer.start();
		wheelSpeedTimer.reset();
		aqHandler.killQueues();
		gyro.reset();
		driverOriented = true;
		emergencyTank = false;
	}
	
	/**
	 * This function is called periodically during teleop mode
	 */
	@Override
	public void teleopPeriodic() {
		// Begin DRIVER CONTROL

		if (INTERFACE_SINGLEDRIVER == false || (INTERFACE_SINGLEDRIVER == true && singleDriverController == 0)) {
			controlWorking = controlDriver;
			
			// Control the robot's drivetrain
			drive();
			
			// Aim down the goal with Limelight
			// TODO only toggles lamp right now. Should aim down target
			if (controlWorking.getRawButtonPressed(Utility.BUTTON_LB)) limelight.toggleLamp();

			// Run the autovolley routine
			if (controlWorking.getRawAxis(Utility.AXIS_RT) > .8) aqHandler.getQueue(QUEUE_SHOOTVOLLEY).queueStart();

			// Reset the gyroscope
			if (controlWorking.getRawButton(Utility.BUTTON_Y) && !emergencyReadjust) gyro.reset();
			
			// Reset the wheels using encoders on the wheels
			if (controlWorking.getRawButton(Utility.BUTTON_X) && !emergencyReadjust) {
				Utility.resetAllWheels(wheel);
				Utility.setAllPIDSetpoints(0, wheel);
			}
			  
			// Reset the wheels based on analog sensing. For emergency purposes only
			if (controlWorking.getRawButton(Utility.BUTTON_B) && !emergencyReadjust) {
				Utility.zeroAllWheelsWithAnalog(wheel);
			}

			// Toggle driver-oriented control
			if (controlWorking.getRawButtonPressed(Utility.BUTTON_A) && !emergencyReadjust) {
				if (driverOriented == true) driverOriented = false; else driverOriented = true;
			}

			// Emergency wheel adjustment mode
			if (controlWorking.getRawButtonPressed(Utility.BUTTON_SELECT) && controlWorking.getRawButtonPressed(Utility.BUTTON_START)) {
				if (emergencyReadjust) {
					emergencyReadjust = false;
					Utility.cleanSlateAllWheels(wheel);
				} else {
					emergencyReadjust = true;
					Utility.powerAllWheels(false, wheel);
				}
			}
			if (emergencyReadjust) readjust();

			// Emergency tank drive mode
			if (controlWorking.getRawButton(Utility.BUTTON_LSTICK) && controlWorking.getRawButton(Utility.BUTTON_RSTICK)) {
				emergencyTank = !emergencyTank;
			}
    	}

		// End DRIVER CONTROL
		// Begin OPERATOR DRIVING

		if (INTERFACE_SINGLEDRIVER == false || (INTERFACE_SINGLEDRIVER == true && singleDriverController == 1)) {
			if (INTERFACE_SINGLEDRIVER == false) controlWorking = controlOperator; else controlWorking = controlDriver;

			// Run the flyheel at the necessary speed to shoot while against the tower and aim the shooter
			if (controlWorking.getRawButton(Utility.BUTTON_Y)) {
				shooter.setFlywheelSpeed(2700);
				shooter.setPivotPosition(0);
			}

			// Run the flywheel at the necessary speed to shoot from anywhere else and aim the shooter accordingly
			if (controlWorking.getRawButton(Utility.BUTTON_X)) {
				shooter.setFlywheelSpeed(3320);
				shooter.setPivotPosition(145);
				// TODO auto aim pivot based on lidar input
				// TODO auto set velocity according to lidar input
			}

			// Simply turn on the flywheel to prepare to shoot
			if (controlWorking.getRawButton(Utility.BUTTON_A)) {
				shooter.setFlywheelSpeed(3200);
			}

			// Kill the shooter
			if (controlWorking.getRawButtonPressed(Utility.BUTTON_B)) {
				shooter.killFlywheel();
			}

			// Execute the logic to continue running the flywheel
			if (shooter.isRunning()) {
				if (shooter.setFlywheelSpeed(shooter.getFlywheelSetpoint())) {
					SmartDashboard.putBoolean("Ready to fire", true);
				} else {
					SmartDashboard.putBoolean("Ready to fire", false);
				}
				// Fine-tune speed
				if (Math.abs(controlWorking.getRawAxis(Utility.AXIS_RSTICKX)) > .25) shooter.setFlywheelSpeed(shooter.getFlywheelSetpoint() + (int) (controlWorking.getRawAxis(Utility.AXIS_RSTICKX) * 6));
			} else {
				// Manually set power to forward or backward of flywheel while it isn't in setpoint mode
				if (controlWorking.getPOV() == 90) shooter.setFlywheelPercentSpeed(.3);
				if (controlWorking.getPOV() == 270) shooter.setFlywheelPercentSpeed(-.3);
			}

			// Run the feeder
			if (controlWorking.getRawAxis(Utility.AXIS_RT) > .1) {
				shooter.runFeeder(-controlWorking.getRawAxis(Utility.AXIS_RT));
			} else if (controlWorking.getRawButton(Utility.BUTTON_RSTICK)) {
				shooter.runFeeder(.75);
			} else shooter.killFeeder();

			// Run the intake wheels
			if (controlWorking.getRawAxis(Utility.AXIS_LT) > .1) {
				intake.setIntakeWheels(controlWorking.getRawAxis(Utility.AXIS_LT));
			} else if (controlWorking.getRawButton(Utility.BUTTON_LSTICK)) {
				intake.setIntakeWheels(-1);
			} else intake.stopIntakeWheels();

			// Run the intake pivot
			if (Math.abs(controlWorking.getRawAxis(Utility.AXIS_LSTICKY)) > .1) {
				intake.setPivot(controlWorking.getRawAxis(Utility.AXIS_LSTICKY) * .8);
			} else intake.setPivot(0);

			// Run the shooter pivot
			if (Math.abs(controlWorking.getRawAxis(Utility.AXIS_RSTICKY)) > .1) {
				shooter.setPivot(controlWorking.getRawAxis(Utility.AXIS_RSTICKY) * .25);
			} else shooter.killPivot();

			// Zero out the shooter pivot encoder
			if (controlWorking.getRawButtonPressed(Utility.BUTTON_RB)) {
				shooter.resetPivotPosition();
			}

			// Run climber motor
			if (controlWorking.getPOV() == 0) {
				climber.set(.5);
			} else if (controlWorking.getPOV() == 180) {
				climber.set(-.5);
			} else climber.set(0);
		}

		// End OPERATOR DRIVING
		// Begin UNIVERSAL FUNCTIONS

		// Toggle drive mode if single driver interface is active
		if (INTERFACE_SINGLEDRIVER == true && controlDriver.getRawButtonPressed(Utility.BUTTON_START) == true) {
			if (singleDriverController == 0) singleDriverController = 1; else singleDriverController = 0;
		}
		
		// Record match time
		matchTime = DriverStation.getInstance().getMatchTime();

		// Change LEDs
		if (driverOriented) leds.setLEDs(Blinkin.C1_HEARTBEAT_MEDIUM); else leds.setLEDs(Blinkin.C2_HEARTBEAT_MEDIUM);

		// Run action queues
		aqHandler.runQueues();
		
		// Dashboard dump
		SmartDashboard.putNumber("ControllerID", singleDriverController);
		SmartDashboard.putNumber("YawAxis", gyro.getYaw());
		SmartDashboard.putBoolean("DriverOriented", driverOriented);
		SmartDashboard.putNumber("Flywheel Velocity", shooter.getFlywheelVelocity());
		SmartDashboard.putNumber("Flywheel Setpoint", shooter.getFlywheelSetpoint());
		SmartDashboard.putNumber("Lidar Dist", lidar.getDistance(Lidar.Unit.INCHES));
		SmartDashboard.putNumber("Flywheel Current", shooter.getFlywheelCurrent());
		SmartDashboard.putNumber("Shooter pivot angle", shooter.getPivotPosition());

		// End UNIVERSAL FUNCTIONS
	}

	@Override
	public void testInit() {
		Utility.cleanSlateAllWheels(wheel);
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		readjust();
		SmartDashboard.putNumber("Lidar Dist", lidar.getDistance(Lidar.Unit.INCHES));
	}
}
