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

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.christmas.2012.ben10watch.used;
// import com.nintendo.gameboy.pikachu;

public class Robot extends TimedRobot {
			
	// CONTROL CONSTANTS
	
	final double CONTROL_SPEEDREDUCTION = .6; 	  			  	// teleop drivetrain inputs are multiplied by this number when turbo is NOT engaged
	final double CONTROL_SPEEDREDUCTION_PRECISION = 3.2;		// teleop drivetrain inputs are divided by this number when precision trigger is engaged
	final double CONTROL_DEADZONE = 0.20;       			    // minimum value before joystick inputs will be considered on the swerves
	final boolean CONTROL_AUTOFAULT_HANDLE = false;				// whether or not the robot will automatically react to a faulty wheel and flip to tank drive

	boolean INTERFACE_SINGLEDRIVER = false;  		  			// whether or not to enable or disable single driver input (press START to switch between controllers)
	//=======================================
	
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
  	//=======================================
  
	// DEFINING HARDWARE

	// Define swerve wheel classes
	static Wheel wheel[] = {
		new Wheel(new TalonSRX(1), new VictorSPX(6), new AnalogInput(1), 0),// front right
		new Wheel(new TalonSRX(0), new VictorSPX(5), new AnalogInput(0), 1),// front left
		new Wheel(new TalonSRX(2), new VictorSPX(7), new AnalogInput(2), 2),// back left
		new Wheel(new TalonSRX(3), new VictorSPX(8), new AnalogInput(3), 3)// back right
	};
	
	// Xbox controllers
	Joystick controlDriver = new Joystick(0);			// the joystick responsible for driving
	Joystick controlOperator = new Joystick(1);			// the joystick responsible for operator controls
	Joystick controlWorking;  							// the controller currently being read from, usually used just for one-driver control
	
	// NavX Constructor
	public static AHRS gyro = new AHRS(SPI.Port.kMXP);

	// Ultrasonic Sensor Constructor
	AnalogInput ultrasonic = new AnalogInput(4);
	ArrayList<Double> usNoise = new ArrayList<Double>(10);
	
	// Action queues
	ActionQueueHandler aqHandler = new ActionQueueHandler(new ActionQueue[] {
		new ActionQueue(true),	// QUEUE_ANGLE
		new ActionQueue(true)	// QUEUE_DRIVESTRAIGHT
	});

	// Reference IDs for action queues
	final int QUEUE_ANGLE = 0;
	final int QUEUE_DRIVESTRAIGHT = 1;
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

				if (reverseRotate) {jRcw = -jRcw;}
				//if (jFwd != 0 && jStr != 0 && jRcw != 0) swerve(jFwd, jStr, jRcw, driverOriented); // the conditional here made the wheels NOT turn to 0,0,0
				// Pull from override information
				if (overrideFWD != 0) jFwd = overrideFWD;
				if (overrideSTR != 0) jStr = overrideSTR;
				if (overrideRCW != 0) jRcw = overrideRCW;
				
				swerve(jFwd, jStr, jRcw, driverOriented);
			}
		} else {
			// Emergency tank drive
			Utility.setAllPIDSetpoints(0, wheel);
			Utility.resetAllWheels(wheel);
			wheel[0].motorDrive.set(ControlMode.PercentOutput, controlWorking.getRawAxis(5));
			wheel[3].motorDrive.set(ControlMode.PercentOutput, controlWorking.getRawAxis(5));
			wheel[2].motorDrive.set(ControlMode.PercentOutput, controlWorking.getRawAxis(1));
			wheel[1].motorDrive.set(ControlMode.PercentOutput, controlWorking.getRawAxis(1));
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

		// If a wheel calculated itself to a value above 1, reduce all wheel speeds
		// TODO play with these lines of code. Try setting each wheel speed to Math.max(wheelSpeed, 1) instead
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
		
		wheel[0].motorDrive.set(ControlMode.PercentOutput, wheelSpeedActual1);
		wheel[1].motorDrive.set(ControlMode.PercentOutput, wheelSpeedActual2);
		wheel[2].motorDrive.set(ControlMode.PercentOutput, wheelSpeedActual3);
		wheel[3].motorDrive.set(ControlMode.PercentOutput, wheelSpeedActual4);
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
		if (controlDriver.getRawAxis(2) > .09) wheel[wheelTune].motorDrive.set(ControlMode.PercentOutput, controlDriver.getRawAxis(2) / 2);
		else if (controlDriver.getRawAxis(3) > .09) wheel[wheelTune].motorDrive.set(ControlMode.PercentOutput, -controlDriver.getRawAxis(3) / 2);
		else wheel[wheelTune].motorDrive.set(ControlMode.PercentOutput, 0);
	}

	/**
	 * Get the value of the ultrasonic sensor, using basic noise reduction techniques to make values more accurate
	 */
	public double getUltrasonicSensor() {
		/*SensorCollection sensor = wheel[1].motorAngle.getSensorCollection();
		return sensor.getAnalogInRaw();*/
		usNoise.add(ultrasonic.getVoltage());
		if (usNoise.size() > 10) usNoise.remove(0);
		double runningAverage = 0;
		double prevNum = 0;
		boolean[] markForRemoval = new boolean[usNoise.size()];

		// Mark values for removal
		for (int i = 0; i < usNoise.size(); i ++) {
			double num = usNoise.get(i);
			if (num >= 1.96 || num - prevNum > 1) {
				markForRemoval[i] = true;
			} else markForRemoval[i] = false;
		}

		// Take average after removing numbers
		for (int i = markForRemoval.length - 1; i < markForRemoval.length; i ++) {
			//if (markForRemoval[i])
		}

		return runningAverage / usNoise.size();
	}

	// <--- ROBOT INITIALIZATION --->
	
	/**
	 * This function is called when the robot is turned on
	 */
	@Override
	public void robotInit() {
		// Feed action queues, they hunger for your command

		aqHandler.getQueue(QUEUE_ANGLE).queueFeed(ActionQueue.Command.TURN_TO_ANGLE, 1, 80, false, 0, 0, 0);

		aqHandler.getQueue(QUEUE_DRIVESTRAIGHT).queueFeed(ActionQueue.Command.DRIVE_STRAIGHT, 1, 120, false, 0, 0, 0);
	}
	
	/**
	 * This function is called immediately when the robot is disabled
	 */
	public void disabledInit() {
		Utility.powerAllWheels(false, wheel);	
		Utility.setAllPIDSetpoints(0, wheel);
		aqHandler.killQueues();
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
			
			// Drive the robot
			drive();
			
			// Start the Action Queue for angling the robot to an angle
			if (controlWorking.getRawButton(Utility.BUTTON_LB)) aqHandler.getQueue(QUEUE_ANGLE).queueStart();
			
			// Start the Action Queue for driving the robot straight
			if (controlWorking.getRawButton(Utility.BUTTON_LSTICK)) aqHandler.getQueue(QUEUE_DRIVESTRAIGHT).queueStart();

			// Reset the gyroscope
			if (controlWorking.getRawButton(Utility.BUTTON_Y)) gyro.reset();
			
			// Reset the wheels
			if (controlWorking.getRawButton(Utility.BUTTON_X)) {
				// Old version of resetting wheels
				Utility.resetAllWheels(wheel);
				Utility.setAllPIDSetpoints(0, wheel);
			}
			  
			// Reset the wheels based on analog sensing. For emergency purposes only
			if (controlWorking.getRawButton(Utility.BUTTON_B)) {
				Utility.zeroAllWheelsWithAnalog(wheel);
			}

			// Toggle driver-oriented control
			if (controlWorking.getRawButtonPressed(Utility.BUTTON_A)) {
				if (driverOriented == true) driverOriented = false; else driverOriented = true;
			}

			// Emergency wheel adjustment mode
			if (controlWorking.getRawButtonPressed(Utility.BUTTON_SELECT) && controlWorking.getRawButtonPressed(Utility.BUTTON_START)) {
				if (emergencyReadjust) {
					emergencyReadjust = false;
					// TODO clean slate protocol here
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
		}

		// End OPERATOR DRIVING
		// Begin UNIVERSAL FUNCTIONS

		// Toggle drive mode if single driver interface is active
		if (INTERFACE_SINGLEDRIVER == true && controlDriver.getRawButton(Utility.BUTTON_START) == true) {
			if (singleDriverController == 0) singleDriverController = 1; else singleDriverController = 0;
		}
		
		// Record match time
		matchTime = DriverStation.getInstance().getMatchTime();

		// Run action queues
		aqHandler.runQueues();
		
		// Dashboard dump
		SmartDashboard.putNumber("ControllerID",singleDriverController);
		SmartDashboard.putNumber("YawAxis", gyro.getYaw());
		SmartDashboard.putBoolean("DriverOriented",driverOriented);
		
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
		/*
		SmartDashboard.putNumber("Joystick y axis", controlDriver.getRawAxis(1));
		
		SmartDashboard.putNumber("Encoder1:", wheel[0].getEncoderPosition());
		SmartDashboard.putNumber("Encoder2:", wheel[1].getEncoderPosition());
		SmartDashboard.putNumber("Encoder3:", wheel[2].getEncoderPosition());
		SmartDashboard.putNumber("Encoder4:", wheel[3].getEncoderPosition());

		SmartDashboard.putBoolean("AnalogIsTripped1:", wheel[0].isZero());
		SmartDashboard.putBoolean("AnalogIsTripped2:", wheel[1].isZero());
		SmartDashboard.putBoolean("AnalogIsTripped3:", wheel[2].isZero());
		SmartDashboard.putBoolean("AnalogIsTripped4:", wheel[3].isZero());
		
		SmartDashboard.putNumber("Analog1:", wheel[0].getRawAnalog());
		SmartDashboard.putNumber("Analog2:", wheel[1].getRawAnalog());
		SmartDashboard.putNumber("Analog3:", wheel[2].getRawAnalog());
		SmartDashboard.putNumber("Analog4:", wheel[3].getRawAnalog());
		*/
		readjust();
		//SmartDashboard.putNumber("Ultrasonic", getUltrasonicSensor());
	}
}
