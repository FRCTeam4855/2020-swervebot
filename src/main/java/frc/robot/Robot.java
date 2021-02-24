/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// FRC TEAM 4855 ROBOT CODE
// 2020 GAME INFINITE RECHARGE

// Swerve-bot code: Robot

//Github test-push 1

package frc.robot;
// package edu.christmas.2012; (DO NOT DELETE; IMPORTANT)

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.christmas.2012.ben10watch.used; (DO NOT DELETE; IMPORTANT)
// import com.nintendo.gameboy.pikachu; (DO NOT DELETE; IMPORTANT)

public class Robot extends TimedRobot {
			
	// CONTROL CONSTANTS
	//=======================================
	final double CONTROL_SPEEDREDUCTION = .5; 	  			  	// teleop drivetrain inputs are multiplied by this number when turbo is NOT engaged
	final double CONTROL_SPEEDREDUCTION_PRECISION = 3.6;		// teleop drivetrain inputs are divided by this number when precision trigger is engaged
	final double CONTROL_DEADZONE = 0.23;       			    // minimum value before joystick inputs will be considered on the swerves
	final boolean CONTROL_AUTOFAULT_HANDLE = false;				// whether or not the robot will automatically react to a faulty wheel and flip to tank drive
	final static double LIMELIGHT_MAXTOLERANCE = 2; 			// the maximum +/- on limelight x to stop attempting to move the robot
	boolean INTERFACE_SINGLEDRIVER = false;  		  			// whether or not to enable or disable single driver input (press START to switch between controllers)
	
	// OTHER CONSTANTS
	final static double ROBOT_WIDTH = 25;
	final static double ROBOT_LENGTH = 24.75;
	final static double ROBOT_R = Math.sqrt(Math.pow(ROBOT_LENGTH, 2) + Math.pow(ROBOT_WIDTH, 2));
	final static double ENC_TO_DEG = 4.6916666;		// formerly 1.158333, then 4.58888 before attempted fix
	final static double ABS_TO_DEG = 11.244444;
	final static double ENC_360 = 1689;				// formerly 417 with original Spark-driven swerves
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
	static double jStr, jFwd, jRcw;
	static double overrideSTR = 0, overrideFWD = 0, overrideRCW = 0;		// set these variables to override drive() values
	// Gradual starts/stops in teleop
	static double wheelSpeedActual1 = 0, wheelSpeedActual2 = 0, wheelSpeedActual3 = 0, wheelSpeedActual4 = 0;
	static Timer wheelSpeedTimer = new Timer();
	// Various variables
	boolean aimMode = false;					// whether or not robot is moving shooter pivot
	boolean aimLobShot = false;					// whether or not robot is shooting from right up against the power port
	boolean aimLockPivot = false;				// whether or not to lock the pivot in place when shooting through PID but never adjusting it
	boolean holdingArm = false;					// whether or not robot is automatically lifting up arm to dump balls as they are fed into shooter
	double aimModePosition = 0;					// the position the robot is attemping to aim the shooter pivot to
	boolean climberDriverOverride = false;		// driver can hit up control to take control of climber
	static boolean showDiagnostics = false;		// show more variables to diagnose issues
	boolean IACbutton = false;

	// AUTONOMOUS VARIABLES
	int a_startType = 1;				// correlates to the driver station you are standing behind
	char a_autoType = 'a';				// correlates to the routine type
	boolean a_truncateRoutine = false;	// truncates the routine to a predefined step
	int a_waitTime = 0;					// waits a prespecified amount of time to start the routine, projected onto dashboard with other controls
  	//=======================================
	
	
	// DEFINING HARDWARE
	//=======================================
	// Define swerve wheel classes
	static Wheel wheel[] = {
		new Wheel(new TalonSRX(1), new CANSparkMax(6, MotorType.kBrushless), new AnalogInput(2), 0),// front right
		new Wheel(new TalonSRX(2), new CANSparkMax(7, MotorType.kBrushless), new AnalogInput(3), 1),// front left
		new Wheel(new TalonSRX(3), new CANSparkMax(8, MotorType.kBrushless), new AnalogInput(1), 2),// back left
		new Wheel(new TalonSRX(4), new CANSparkMax(9, MotorType.kBrushless), new AnalogInput(0), 3)// back right	
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
	UsbCamera camForward;
	UsbCamera camClimb;
	VideoSink server;
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
		new ActionQueue(true),	// QUEUE_AUTONOMOUS_1C
		new ActionQueue(true),	// QUEUE_AUTONOMOUS_2A
		new ActionQueue(true),	// QUEUE_AUTONOMOUS_2B
		new ActionQueue(true),	// QUEUE_AUTONOMOUS_2C
		new ActionQueue(true),	// QUEUE_AUTONOMOUS_3A
		new ActionQueue(true),	// QUEUE_AUTONOMOUS_3B
		new ActionQueue(true),	// QUEUE_AUTONOMOUS_4A
		new ActionQueue(true),	// QUEUE_AUTONOMOUS_4B
		new ActionQueue(true)	// MADDEN_SPINMOVE
	};

	// Reference IDs for action queues
	final int QUEUE_ANGLE = 0;
	final int QUEUE_DRIVESTRAIGHT = 1;
	final int QUEUE_SHOOTVOLLEY = 2;
	final int QUEUE_LIMELIGHTANGLE = 3;

	final int QUEUE_AUTONOMOUS_1A = 4;	// basic clip release routine
	final int QUEUE_AUTONOMOUS_1B = 5;
	final int QUEUE_AUTONOMOUS_1C = 6;
	final int QUEUE_AUTONOMOUS_2A = 7;	// basic clip release routine
	final int QUEUE_AUTONOMOUS_2B = 8;
	final int QUEUE_AUTONOMOUS_2C = 9;
	final int QUEUE_AUTONOMOUS_3A = 10;	// basic clip release routine
	final int QUEUE_AUTONOMOUS_3B = 11;
	final int QUEUE_AUTONOMOUS_4A = 12;	// drive forward
	final int QUEUE_AUTONOMOUS_4B = 13;
	final int MADDEN_SPINMOVE = 14;
	//=======================================

	// End of variable definitions

	/**
	 * Drives the robot and invokes the Wheel method process(). If emergency tank mode is enabled, then the swerve wheels will behave as two pairs of tank wheels.
	 */
	public void drive() {
		if (!emergencyTank) {
			if (!emergencyReadjust && (!controlWorking.getRawButton(Utility.BUTTON_LSTICK) && !controlWorking.getRawButton(Utility.BUTTON_RSTICK))) {
				// Drive the robot, will adjust driverOriented based on toggled input
				jFwd = -controlWorking.getRawAxis(Utility.AXIS_LSTICKY);if (Math.abs(jFwd) < CONTROL_DEADZONE) jFwd = 0;
				if (!controlWorking.getRawButton(Utility.BUTTON_RB) && controlWorking.getRawAxis(Utility.AXIS_LT) < .7) jFwd *= CONTROL_SPEEDREDUCTION;
				if (controlWorking.getRawAxis(Utility.AXIS_LT) >= .7) jFwd /= CONTROL_SPEEDREDUCTION_PRECISION;

				jStr = controlWorking.getRawAxis(Utility.AXIS_LSTICKX);if (Math.abs(jStr) < CONTROL_DEADZONE) jStr = 0;
				if (!controlWorking.getRawButton(Utility.BUTTON_RB) && controlWorking.getRawAxis(Utility.AXIS_LT) < .7) jStr *= CONTROL_SPEEDREDUCTION;
				if (controlWorking.getRawAxis(Utility.AXIS_LT) >= .7) jStr /= CONTROL_SPEEDREDUCTION_PRECISION;

				jRcw = controlWorking.getRawAxis(Utility.AXIS_RSTICKX);if (Math.abs(jRcw) < CONTROL_DEADZONE) jRcw = 0;
				if (!controlWorking.getRawButton(Utility.BUTTON_RB) && controlWorking.getRawAxis(Utility.AXIS_LT) < .7) jRcw *= CONTROL_SPEEDREDUCTION;
				if (controlWorking.getRawAxis(Utility.AXIS_LT) >= .7) jRcw /= CONTROL_SPEEDREDUCTION_PRECISION;

				if (reverseRotate) jRcw = -jRcw;
				boolean turnWheels;					// whether or not the wheels should attempt to spin to a new angle
				// if ((jFwd != 0 && jStr != 0 && jRcw != 0)) turnWheels = false; else turnWheels = true;
				
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
				//System.out.println("jRcw " + jRcw);
				if (!controlDriver.getRawButton(Utility.BUTTON_X)) swerve(jFwd, jStr, jRcw, driverOriented);
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
		// Local method variables
		//System.out.println("RCW  " + RCW);
		double a, b, c, d, max, temp, rads; 
		double encoderSetpointA, encoderSetpointB, encoderSetpointC, encoderSetpointD;
		double wheelSpeed1, wheelSpeed2, wheelSpeed3, wheelSpeed4;

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
			if (wheelSpeed1 - wheelSpeedActual1 > 0.15) {wheelSpeedActual1 += 0.15;} else if (wheelSpeed1 - wheelSpeedActual1 < -0.15) {wheelSpeedActual1 -= 0.15;} else {wheelSpeedActual1 = wheelSpeed1;}
			if (wheelSpeed2 - wheelSpeedActual2 > 0.15) {wheelSpeedActual2 += 0.15;} else if (wheelSpeed2 - wheelSpeedActual2 < -0.15) {wheelSpeedActual2 -= 0.15;} else {wheelSpeedActual2 = wheelSpeed2;}
			if (wheelSpeed3 - wheelSpeedActual3 > 0.15) {wheelSpeedActual3 += 0.15;} else if (wheelSpeed3 - wheelSpeedActual3 < -0.15) {wheelSpeedActual3 -= 0.15;} else {wheelSpeedActual3 = wheelSpeed3;}
			if (wheelSpeed4 - wheelSpeedActual4 > 0.15) {wheelSpeedActual4 += 0.15;} else if (wheelSpeed4 - wheelSpeedActual4 < -0.15) {wheelSpeedActual4 -= 0.15;} else {wheelSpeedActual4 = wheelSpeed4;}
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

		// Configure USB camera 1
		camForward = CameraServer.getInstance().startAutomaticCapture(0);
		camForward.setBrightness(20);
		camForward.setExposureManual(50);
		camForward.setResolution(160, 120);
		camForward.setFPS(15);

		// Configure USB camera 2
		camClimb = CameraServer.getInstance().startAutomaticCapture(1);
		camClimb.setBrightness(20);
		camClimb.setExposureManual(50);
		camClimb.setResolution(160, 120);
		camClimb.setFPS(15);

		// Configure server to only watch the normal camera
		server = CameraServer.getInstance().getServer();
		server.setSource(camForward);
		
		aqHandler = new ActionQueueHandler(aqArray);

		// Feed action queues, they hunger for your command

		aqHandler.getQueue(QUEUE_ANGLE).queueFeed(ActionQueue.Command.TURN_TO_ANGLE, 0, 2, false, 112.5, 0, 0);	// automatically turn to the angle of the bar

		aqHandler.getQueue(QUEUE_DRIVESTRAIGHT).queueFeed(ActionQueue.Command.DRIVE_STRAIGHT, 0, 3, false, .3, 0, 0);

		aqHandler.getQueue(QUEUE_LIMELIGHTANGLE).queueFeed(ActionQueue.Command.ANGLE_TO_LIMELIGHT_X, 0, 1, false, 0, 0, 0);

		// Autonomous Routine 1A - Basic clip release
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1A).queueFeed(ActionQueue.Command.INTAKE_PIVOT, 0, 1.25, true, .5, 0, 0);		// bring down intake pivot
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1A).queueFeed(ActionQueue.Command.RUN_INTAKE_WHEELS, 1, 3, true, 1, 0, 0);		// fling ball into robot
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1A).queueFeed(ActionQueue.Command.RUN_FLYWHEEL, .7, 3.9, true, 2990, 0, 0);		// turn on flywheel
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1A).queueFeed(ActionQueue.Command.SHOOTER_PIVOT, 0, 5, true, 740, 0, 0);		// shooter pivot to correct position
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1A).queueFeed(ActionQueue.Command.WAIT_FOR_SENSOR, 1.1, 1.2, false, 1, 0, 0);	// wait to reach speed
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1A).queueFeed(ActionQueue.Command.FEED_BALL, 1.5, 4.1, true, 0, 0, 0);			// feed balls into shooter
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1A).queueFeed(ActionQueue.Command.SWERVE, 7, 8, true, -.18, 0, 0);				// back up
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1A).queueFeed(ActionQueue.Command.SWERVE, 8.1, 9, true, 0, 0, 0);				// stop driving

		// Autonomous Routine 1B - Start in front of station 1, shoot, pick up balls directly behind machine and shoot again
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1B).queueFeed(ActionQueue.Command.INTAKE_PIVOT, 0, 1.25, true, .5, 0, 0);		// bring down intake pivot
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1B).queueFeed(ActionQueue.Command.RUN_INTAKE_WHEELS, 1, 3, true, 1, 0, 0);
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1B).queueFeed(ActionQueue.Command.RUN_FLYWHEEL, .7, 3.9, true, 0, 1, 0);		// turn on flywheel
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1B).queueFeed(ActionQueue.Command.WAIT_FOR_SENSOR, 1.1, 1.2, false, 1, 0, 0);	// wait to reach speed
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1B).queueFeed(ActionQueue.Command.FEED_BALL, 1.5, 4.1, true, 0, 0, 0);			// feed balls into shooter
		// At this point, the first clip has been unloaded
		// TODO *something something use color sensor to count balls*
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1B).queueFeed(ActionQueue.Command.SWERVE, 3.8, 7, true, 0.22, 0, 0);	// turn around and drive
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1B).queueFeed(ActionQueue.Command.RUN_INTAKE_WHEELS, 4.2, 7, true, 1, 0, 0);	// intake balls
		// TODO *something something check drive encoders for distance*
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1B).queueFeed(ActionQueue.Command.SWERVE, 7, 7.05, true, 0, 0, 0);
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1B).queueFeed(ActionQueue.Command.TURN_TO_ANGLE, 7, 9, true, 0, 0, 0);			// turn robot around
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1B).queueFeed(ActionQueue.Command.ANGLE_TO_LIMELIGHT_X, 8.6, 9, true, 1, 0, 0);	// angle to target
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1B).queueFeed(ActionQueue.Command.RUN_FLYWHEEL, 7, 12, true, 3320, 0, 0);		// turn on flywheel
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1B).queueFeed(ActionQueue.Command.WAIT_FOR_SENSOR, 7.3, 7.31, false, 1, 0, 0);	// wait to reach speed
		aqHandler.getQueue(QUEUE_AUTONOMOUS_1B).queueFeed(ActionQueue.Command.FEED_BALL, 8, 11.5, true, 0, 0, 0);			// feed balls into shooter

		// Autonomous Routine 4A - Autocross from any location
		aqHandler.getQueue(QUEUE_AUTONOMOUS_4A).queueFeed(ActionQueue.Command.DRIVE_STRAIGHT, 1, 2.4, false, -.2, 0, 0);
		//aqHandler.getQueue(QUEUE_AUTONOMOUS_4A).queueFeed(ActionQueue.Command.WAIT_FOR_SENSOR, 1.01, 1.02, false, 3, 3000, 10);
		aqHandler.getQueue(QUEUE_AUTONOMOUS_4A).queueFeed(ActionQueue.Command.SWERVE, 2.5, 15, false, 0, 0, 0);

		// Put default auto chooser values to the dashboard
		SmartDashboard.putNumber("Auto: Station #", a_startType);
		SmartDashboard.putString("Auto: Routine Type", String.valueOf(a_autoType));
		SmartDashboard.putBoolean("Auto: Truncate", a_truncateRoutine);
		SmartDashboard.putNumber("Auto: Wait Time", a_waitTime);
	}
	
	/**
	 * This function is called immediately when the robot is disabled
	 */
	@Override
	public void disabledInit() {
		Utility.cleanSlateAllWheels(wheel);
		Utility.powerAllWheels(false, wheel);	
		Utility.setAllPIDSetpoints(0, wheel);
		shooter.killFlywheel();
		shooter.killFeeder();
		aqHandler.killQueues();
		intake.stop();
	}

	@Override
	public void disabledPeriodic() {
		// Display autonomous controls
		/*SmartDashboard.putNumber("Auto: Station #", a_startType);
		SmartDashboard.putString("Auto: Routine Type", String.valueOf(a_autoType));
		SmartDashboard.putBoolean("Auto: Truncate", a_truncateRoutine);*/

		a_startType = (int) SmartDashboard.getNumber("Auto: Station #", 1);
		a_autoType = SmartDashboard.getString("Auto: Routine Type", "a").toLowerCase().charAt(0);
		a_truncateRoutine = SmartDashboard.getBoolean("Auto: Truncate", a_truncateRoutine);
		a_waitTime = (int) SmartDashboard.getNumber("Auto: Wait Time", 0);

		// Keep Limelight lamps turned off
		limelight.turnOffLamp();
	}
	
	/**
	 * This function is called immediately when autonomous begins
	 */
	@Override
	public void autonomousInit() {
		Utility.cleanSlateAllWheels(wheel);
		wheelSpeedTimer.start();
		wheelSpeedTimer.reset();
		aqHandler.killQueues();
		gyro.reset();
		driverOriented = true;
		emergencyTank = false;
		int routineNumber = QUEUE_AUTONOMOUS_4A;	// fallback auto routine

		// Pick the routine based on dashboard input
		switch (a_autoType) {
			case 'a':
				switch (a_startType) {
					case 1:
						routineNumber = QUEUE_AUTONOMOUS_1A;
						break;
					case 2:
						routineNumber = QUEUE_AUTONOMOUS_2A;
						break;
					case 3:
						routineNumber = QUEUE_AUTONOMOUS_3A;
						break;
					case 4:
						routineNumber = QUEUE_AUTONOMOUS_4A;
						break;
				}
				break;
			case 'b':
				switch (a_startType) {
					case 1:
						routineNumber = QUEUE_AUTONOMOUS_1B;
						break;
					case 2:
						routineNumber = QUEUE_AUTONOMOUS_2B;
						break;
					case 3:
						routineNumber = QUEUE_AUTONOMOUS_3B;
						break;
					case 4:
						routineNumber = QUEUE_AUTONOMOUS_4B;
						break;
				}
				break;
			case 'c':
				switch (a_startType) {
					case 1:
						routineNumber = QUEUE_AUTONOMOUS_1C;
						break;
					case 2:
						routineNumber = QUEUE_AUTONOMOUS_2C;
						break;
				}
				break;

		}

		if (a_waitTime > 0) aqHandler.getQueue(routineNumber).queueFeed(ActionQueue.Command.WAIT_FOR_SENSOR, 0, 0, false, 0, a_waitTime, 0);
		aqHandler.getQueue(routineNumber).queueStart();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		aqHandler.runQueues();
		SmartDashboard.putNumber("Shooter pivot angle", shooter.getPivotPosition());
		leds.setLEDs(Blinkin.FIRE_LARGE);
	}
	
	/**
	 * This function is called when teleop begins
	 */
	public void teleopInit() {
		// TODO check that transistion between auton and teleop is seamless without resetting a bunch of wheel positions
		if (!DriverStation.getInstance().isFMSAttached()) {
			Utility.cleanSlateAllWheels(wheel);
			wheelSpeedTimer.start();
			wheelSpeedTimer.reset();
			aqHandler.killQueues();
			gyro.reset();
			driverOriented = true;
			emergencyTank = false;
		} else {
			Utility.powerAllWheels(true, wheel);
		}
	}
	
	/**
	 * This function is called periodically during teleop mode
	 */
	@Override
	public void teleopPeriodic() {
		// Begin DRIVER CONTROL
		//wheel[0].process();
		
		if (INTERFACE_SINGLEDRIVER == false || (INTERFACE_SINGLEDRIVER == true && singleDriverController == 0)) {
			controlWorking = controlDriver;
			
			// Control the robot's drivetrain
			drive();

			// Change LEDs
			if (driverOriented) leds.setLEDs(Blinkin.C1_HEARTBEAT_MEDIUM); else leds.setLEDs(Blinkin.C2_HEARTBEAT_MEDIUM);

			// Override and operate the climber
			if (controlWorking.getPOV() == 180) {
				if (climberDriverOverride) server.setSource(camForward); else server.setSource(camClimb);
				climberDriverOverride = !climberDriverOverride;
			}
			if (climberDriverOverride) {
				leds.setLEDs(Blinkin.COLORWAVES_RAINBOWPALETTE);
				if (Math.abs(controlWorking.getRawAxis(Utility.AXIS_RSTICKY)) > .1) {
					climber.set(controlWorking.getRawAxis(Utility.AXIS_RSTICKY) * .75);
				} else climber.set(0);
			}
			
			// Aim down the goal with Limelight
			if (controlWorking.getRawButton(Utility.BUTTON_LB)) {
				limelight.turnOnLamp();
				ActionQueueHandler.queueAngle_To_Limelight_X(0);
			} else {
				limelight.turnOffLamp();
				overrideRCW = 0;
			}

			// Reset the gyroscope
			if (controlWorking.getRawButton(Utility.BUTTON_Y) && !emergencyReadjust) gyro.reset();
			
			// Reset the wheels using encoders on the wheels
			if (controlWorking.getRawButton(Utility.BUTTON_X) && !emergencyReadjust) {
				Utility.resetAllWheels(wheel);
				Utility.setAllPIDSetpoints(0, wheel);
			}
			  
			// Reset the wheels based on analog sensing. For emergency purposes only
			if (controlWorking.getRawButton(Utility.BUTTON_B) && !emergencyReadjust) {
				//Utility.zeroAllWheelsWithAnalog(wheel); analogs are not currently plugged in. disabled for now
			}

			// Toggle driver-oriented control
			if (controlWorking.getRawButtonPressed(Utility.BUTTON_A) && !emergencyReadjust) {
				if (driverOriented == true) driverOriented = false; else driverOriented = true;
			}

			// TODO remove this
			// Run angle routine (TEST)
			if (controlWorking.getRawButtonPressed(Utility.BUTTON_START)) aqHandler.getQueue(QUEUE_ANGLE).queueStart();

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
			if (controlWorking.getRawButton(Utility.BUTTON_Y) && !IACbutton) {
				shooter.setFlywheelSpeed(2400);
				shooter.setPivotPosition(130);
				aimMode = true;
				aimLobShot = true;
				aimModePosition = 130;
			}

			// Run the flywheel at the necessary speed to shoot from anywhere else and aim the shooter accordingly
			if (controlWorking.getRawButton(Utility.BUTTON_X) && !IACbutton) {
				shooter.setFlywheelSpeed(3000);	// default speed position
				shooter.setPivotPosition(1000);
				aimMode = true;
				aimLobShot = false;
				aimModePosition = 1000; // default starting position
			}

			

			// Run the flywheel to shoot optimally in the trench run
			if (controlWorking.getRawButton(Utility.BUTTON_RB) && !IACbutton) {
				shooter.setFlywheelSpeed(3265);	// default speed position
				shooter.setPivotPosition(800);
				aimMode = true;
				aimLockPivot = true;
				aimModePosition = 800; // default starting position
			}

			// Kill the shooter
			if (controlWorking.getRawButtonPressed(Utility.BUTTON_B)) {
				shooter.killFlywheel();
				aimMode = false;
				aimLobShot = false;
				aimLockPivot = false;
			}

			// Simply turn on the flywheel to prepare to shoot
			if (controlWorking.getRawButton(Utility.BUTTON_A) && !IACbutton) {
				shooter.setFlywheelSpeed(3320);
			}

			// Toggle white "READY" light
		/*if (controlWorking.getRawButton(Utility.BUTTON_LB)) {
			leds.setLEDs(Blinkin.WHITE);
			IACbutton = true;
		} */
		
		// Run the flywheel to shoot optimally in the GREEN Zone
		if (controlWorking.getRawButton(Utility.BUTTON_LB) && controlWorking.getRawButton(Utility.BUTTON_A)) {
			shooter.setFlywheelSpeed(2400);
			shooter.setPivotPosition(504);
			aimMode = true;
			aimModePosition = 504;
			leds.setLEDs(Blinkin.GREEN);
			IACbutton = true;
		}
		// Run the flywheel to shoot optimally in the YELLOW Zone
		if (controlWorking.getRawButton(Utility.BUTTON_LB) && controlWorking.getRawButton(Utility.BUTTON_Y)) {
			shooter.setFlywheelSpeed(2400);
			shooter.setPivotPosition(758);
			aimMode = true;
			aimModePosition = 758;
			leds.setLEDs(Blinkin.YELLOW);
			IACbutton = true;
		}
		// Run the flywheel to shoot optimally in the BLUE Zone
		if (controlWorking.getRawButton(Utility.BUTTON_LB) && controlWorking.getRawButton(Utility.BUTTON_X)) {
			shooter.setFlywheelSpeed(2400);
			shooter.setPivotPosition(767);
			aimMode = true;
			aimModePosition = 767;
			leds.setLEDs(Blinkin.SKY_BLUE);
			IACbutton = true;
		}
		// Run the flywheel to shoot optimally in the RED Zone
		if (controlWorking.getRawButton(Utility.BUTTON_LB) && controlWorking.getRawButton(Utility.BUTTON_B)) {
			shooter.setFlywheelSpeed(3073);
			shooter.setPivotPosition(877);
			aimMode = true;
			aimModePosition = 877;
			leds.setLEDs(Blinkin.RED);
			IACbutton = true;
		}
		

			// Execute the logic to continue running the flywheel
			if (shooter.isRunning()) {
				if (shooter.setFlywheelSpeed(shooter.getFlywheelSetpoint())) {
					SmartDashboard.putBoolean("Ready to fire", true);
					leds.setLEDs(Blinkin.SHOT_BLUE);
				} else {
					SmartDashboard.putBoolean("Ready to fire", false);
				}
				// Fine-tune speed
				if (Math.abs(controlWorking.getRawAxis(Utility.AXIS_RSTICKX)) > .25) {
					aimMode = false;	// something was manually changed so stay put
					shooter.setFlywheelSpeed(shooter.getFlywheelSetpoint() + (int) (controlWorking.getRawAxis(Utility.AXIS_RSTICKX) * 6));
				}
				// Auto-aiming
				if (aimMode && !aimLobShot && !aimLockPivot) {
					shooter.setFlywheelSpeed(shooter.getVelocityFromDistance(lidar.getDistance(Lidar.Unit.INCHES)));
					double setAim = shooter.getPivotPositionFromDistance(lidar.getDistance(Lidar.Unit.INCHES));
					aimModePosition = setAim;
				}
			} else {
				// Manually set power to forward or backward of flywheel while it isn't in setpoint mode
				SmartDashboard.putBoolean("Ready to fire", false);
				if (controlWorking.getPOV() == 90) 
					shooter.setFlywheelPercentSpeed(-.5);
				else if (controlWorking.getPOV() == 270) 
					shooter.setFlywheelPercentSpeed(.5);
				else shooter.setFlywheelPercentSpeed(0);
			}

			// Run the shooter pivot
			if (Math.abs(controlWorking.getRawAxis(Utility.AXIS_RSTICKY)) > .1) {
				shooter.setPivot(controlWorking.getRawAxis(Utility.AXIS_RSTICKY) * .25);
				aimMode = false;
				aimLockPivot = false;
			} else shooter.killPivot();

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
				intake.setPivot(controlWorking.getRawAxis(Utility.AXIS_LSTICKY) * .5);
				//intake.setPivotPosition(400);
			} else if (!aimMode) intake.setPivot(0);

			// Zero out the shooter pivot encoder
			if (controlWorking.getRawButtonPressed(Utility.BUTTON_START)) {
				shooter.resetPivotPosition();
			}

			// Zero out the intake pivot encoder
			if (controlWorking.getRawButtonPressed(Utility.BUTTON_SELECT)) {
				intake.resetPivotPosition();
			}

			// Run climber motor
			if (!climberDriverOverride) {
				if (controlWorking.getPOV() == 0) {
					climber.set(.5);
				} else if (controlWorking.getPOV() == 180) {
					climber.set(-.5);
				} else climber.set(0);
			}

			// Aim down pivot if aimMode is active
			if (aimMode || aimLockPivot) {
				shooter.setPivotPosition(aimModePosition);
			}
		}

		
		// Begin OPERATOR CONTROLS: Interstellar Accuracy Challenge 

		// Toggle white "READY" light
		if (controlWorking.getRawButton(Utility.BUTTON_LB)) {
			leds.setLEDs(Blinkin.WHITE);
		} 

		// Run the flywheel to shoot optimally in the GREEN Zone
		if (controlWorking.getRawButton(Utility.BUTTON_LB) && controlWorking.getRawButton(Utility.BUTTON_A)) {
			shooter.setFlywheelSpeed(2400);
			shooter.setPivotPosition(130);
			aimMode = true;
			aimModePosition = 130;
			leds.setLEDs(Blinkin.GREEN);
		}
		// Run the flywheel to shoot optimally in the YELLOW Zone
		if (controlWorking.getRawButton(Utility.BUTTON_LB) && controlWorking.getRawButton(Utility.BUTTON_Y)) {
			shooter.setFlywheelSpeed(2400);
			shooter.setPivotPosition(130);
			aimMode = true;
			aimModePosition = 130;
			leds.setLEDs(Blinkin.YELLOW);
		}
		// Run the flywheel to shoot optimally in the BLUE Zone
		if (controlWorking.getRawButton(Utility.BUTTON_LB) && controlWorking.getRawButton(Utility.BUTTON_X)) {
			shooter.setFlywheelSpeed(2400);
			shooter.setPivotPosition(130);
			aimMode = true;
			aimModePosition = 130;
			leds.setLEDs(Blinkin.SKY_BLUE);
		}
		// Run the flywheel to shoot optimally in the RED Zone
		if (controlWorking.getRawButton(Utility.BUTTON_LB) && controlWorking.getRawButton(Utility.BUTTON_B)) {
			shooter.setFlywheelSpeed(2400);
			shooter.setPivotPosition(130);
			aimMode = true;
			aimModePosition = 130;
			leds.setLEDs(Blinkin.RED);
		}

		// End OPERATOR DRIVING

				// Begin OPERATOR CONTROLS: Interstellar Accuracy Challenge 

		
		// End OPERATOR DRIVING

		// Begin UNIVERSAL FUNCTIONS

		// Toggle drive mode if single driver interface is active
		if (INTERFACE_SINGLEDRIVER == true && controlDriver.getRawButtonPressed(Utility.BUTTON_START) == true) {
			if (singleDriverController == 0) singleDriverController = 1; else singleDriverController = 0;
		}
		
		// Record match time
		matchTime = DriverStation.getInstance().getMatchTime();

		// Run action queues
		aqHandler.runQueues();

		// Change lights
		if (controlDriver.getRawButton(Utility.BUTTON_LB)) leds.setLEDs(Blinkin.LIGHTCHASE_RED);
		if (emergencyReadjust) leds.setLEDs(Blinkin.HOT_PINK);
		
		// Dashboard dump
		if (showDiagnostics) {
			SmartDashboard.putNumber("ControllerID", singleDriverController);
			
			SmartDashboard.putNumber("Flywheel Current", shooter.getFlywheelCurrent());
		}

		SmartDashboard.putNumber("Intake pivot angle", intake.getPivotPosition()); // temporary for testing

		SmartDashboard.putNumber("YawAxis", gyro.getYaw()); 
		// add a potentional "pitch" display for a robotOrieted-relative  
		SmartDashboard.putBoolean("DriverOriented", driverOriented);
		SmartDashboard.putNumber("Flywheel Velocity", shooter.getFlywheelVelocity());
		SmartDashboard.putNumber("Flywheel Setpoint", shooter.getFlywheelSetpoint());
		SmartDashboard.putNumber("Lidar Dist", lidar.getDistance(Lidar.Unit.INCHES));
		SmartDashboard.putNumber("Shooter pivot angle", shooter.getPivotPosition());

		shooter.sensePowerCells();
		//SmartDashboard.putNumber("Power cells", shooter.getPowerCellsLeft());
		
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
		//wheel[0].process();
		leds.setLEDs(Blinkin.WHITE);
		readjust();
		//SmartDashboard.putNumber("Lidar Dist", lidar.getDistance(Lidar.Unit.INCHES));
	}
}