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

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.christmas.2012.ben10watch.used;
// import com.nintendo.gameboy.pikachu;

public class Robot extends TimedRobot {
			
	// CONTROL CONSTANTS
	
	final double CONTROL_SPEEDREDUCTION = .6; 	  			  // teleop drivetrain inputs are multiplied by this number when turbo is NOT engaged
	final double CONTROL_SPEEDREDUCTION_PRECISION = 3.2;	// teleop drivetrain inputs are divided by this number when precision trigger is engaged
	final double CONTROL_DEADZONE = 0.21;       			    // minimum value before joystick inputs will be considered on the swerves

	final boolean INTERFACE_SINGLEDRIVER = false;  		  	// whether or not to enable or disable single driver input (press START to switch between controllers)
	//=======================================
	
	// OTHER CONSTANTS

	final static double ROBOT_WIDTH = 29;
	final static double ROBOT_LENGTH = 29;
	final static double ROBOT_R = Math.sqrt(Math.pow(ROBOT_LENGTH,2)+Math.pow(ROBOT_WIDTH,2));
	final static double ENC_TO_DEG = 1.158333;
	final static double ABS_TO_DEG = 11.244444;
	final static double ENC_360 = 417;
	final static double IN_TO_ENC = 10.394;
	// buttons
	final static int BUTTON_A = 1;
	final static int BUTTON_B = 2;
	final static int BUTTON_X = 3;
	final static int BUTTON_Y = 4;
	final static int BUTTON_LB = 5;
	final static int BUTTON_RB = 6;
	final static int BUTTON_SELECT = 7;
	final static int BUTTON_START = 8;
	final static int BUTTON_LSTICK = 9;
	final static int BUTTON_RSTICK = 10;

	// BEGINNING VARIABLES
	
	int wheelTune = 0; 								        // Remembers what wheel we are tweaking in test mode
	int singleDriverController = 0; 				  // port number of controller to operate
	boolean emergencyTank = false; 					  // True if the robot is in emergency tank drive mode
	boolean reverseRotate = false; 					  // ?????
	boolean driverOriented = true; 				  	// true = driver oriented, false = robot oriented
	static double matchTime = 0;					    // the calculated match time from the driver station
	boolean emergencyReadjust = false;				// if hell has come to earth and you need to manually adjust wheels during a match, this will be enabled
	String playType = "MATCH";					    	// whether to act as if in a match ("MATCH") or testing ("TEST")
	// All for calculating wheel speed/angle, if you need to read from a motor don't pull from these
	static double a, b, c, d, max, temp, rads; 
	static double encoderSetpointA, encoderSetpointB, encoderSetpointC, encoderSetpointD;
	static double jStr, jFwd, jRcw;
	static double wheelSpeed1, wheelSpeed2, wheelSpeed3, wheelSpeed4;
	// Gradual starts/stops in teleop
	static double wheelSpeedActual1 = 0, wheelSpeedActual2 = 0, wheelSpeedActual3 = 0, wheelSpeedActual4 = 0;
	static Timer wheelSpeedTimer = new Timer();
  //=======================================
  
	// DEFINING HARDWARE

	// Magnetic encoders
	static Encoder encoderAngle[] = {
		new Encoder(0,1),
		new Encoder(2,3),
		new Encoder(6,7),
		new Encoder(4,5)
	};

	// Define swerve wheel classes
	static Wheel wheel[] = {
		new Wheel(encoderAngle[0]),
		new Wheel(encoderAngle[1]),
		new Wheel(encoderAngle[2]),
		new Wheel(encoderAngle[3])
	};
	
	// Xbox controllers
	Joystick controlDriver = new Joystick(0);			// the joystick responsible for driving
	Joystick controlOperator = new Joystick(1);			// the joystick responsible for operator controls
	Joystick controlWorking;  							// the controller currently being read from, usually used just for one-driver control
	
	// NavX Constructor
	private static AHRS ahrs = new AHRS(SPI.Port.kMXP);
	
	// All motors
	static Spark motorAngle[] = { // Directional motors
		new Spark(3),
		new Spark(7),
		new Spark(6),
		new Spark(2)
	};
	
	static Spark motorDrive[] = { // Movement motors
		new Spark(1),
		new Spark(5),
		new Spark(4),
		new Spark(0)
	};
	//=======================================
	
	// PID LOOPS AND ROUTINES

	// These control the steering motors using the mers (?? idk what mers is)
	static PIDController PIDdrive[] = {
		new PIDController(0.035,0,0.01,encoderAngle[0],motorAngle[0]),
		new PIDController(0.035,0,0.01,encoderAngle[1],motorAngle[1]),
		new PIDController(0.035,0,0.01,encoderAngle[2],motorAngle[2]),
		new PIDController(0.035,0,0.01,encoderAngle[3],motorAngle[3])
	};
	
	// Action queues
	ActionQueue actionQueues[] = {
		new ActionQueue()
	};

	// Reference IDs for action queues
	final int QUEUE_TEST = 0;
	//=======================================

	// End of variable definitions
	// <--- ROBOT INITIALIZATION --->
	
	/**
	 * This function is called when the robot is turned on
	 */
	@Override
	public void robotInit() {
		// Configure swerve wheel PID loops
		PIDdrive[0].setOutputRange(-1, 1);
		PIDdrive[1].setOutputRange(-1, 1);
		PIDdrive[2].setOutputRange(-1, 1);
		PIDdrive[3].setOutputRange(-1, 1);

		// Feed action queues, they hunger for your command
		// Test, does basically nothing
		actionQueues[QUEUE_TEST].queueFeed(ActionQueue.Command.SWERVE,1,50,false,.4,0,0);
	}
	
	/**
	 * This function is called immediately when the robot is disabled
	 */
	public void disabledInit() {
		setAllPIDControllers(PIDdrive, false);	
		setAllPIDSetpoints(PIDdrive, 0);
		killQueues(actionQueues);
	}
	
	/**
	 * This function is called immediately when autonomous begins
	 */
	@Override
	public void autonomousInit() {
		wheelSpeedTimer.start();
		wheelSpeedTimer.reset();
		ahrs.reset();
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
		setAllPIDControllers(PIDdrive, true);
		wheelSpeedTimer.start();
		wheelSpeedTimer.reset();
		encoderAngle[0].reset();encoderAngle[1].reset();encoderAngle[2].reset();encoderAngle[3].reset();
		resetAllWheels();killQueues(actionQueues);
		ahrs.reset();
		driverOriented = true;
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
			
			// Reset the gyroscope
			if (controlWorking.getRawButton(BUTTON_Y)) ahrs.reset();
			
			// Reset the wheels
			if (controlWorking.getRawButton(BUTTON_X)) {
				resetAllWheels();
				setAllPIDSetpoints(PIDdrive, 0);
      		}
      
			// Toggle driver-oriented control
			if (controlWorking.getRawButtonPressed(BUTTON_A)) {
				if (driverOriented == true) driverOriented = false; else driverOriented = true;
			}

			// Emergency wheel adjustment mode
			if (controlWorking.getRawButtonPressed(BUTTON_SELECT) && controlWorking.getRawButtonPressed(BUTTON_START)) {
				if (emergencyReadjust) {
					emergencyReadjust = false;
					setAllPIDControllers(PIDdrive, true);
					resetAllWheels();
					setAllPIDSetpoints(PIDdrive, 0);
					encoderAngle[0].reset();encoderAngle[1].reset();encoderAngle[2].reset();encoderAngle[3].reset();
				} else {
					emergencyReadjust = true;
					setAllPIDControllers(PIDdrive, false);
				}
			}
			if (emergencyReadjust) readjust();
    	}

		// End DRIVER CONTROL
		// Begin OPERATOR DRIVING

		if (INTERFACE_SINGLEDRIVER == false || (INTERFACE_SINGLEDRIVER == true && singleDriverController == 1)) {
			if (INTERFACE_SINGLEDRIVER == false) controlWorking = controlOperator; else controlWorking = controlDriver;
		}

		// End OPERATOR DRIVING
		// Begin UNIVERSAL FUNCTIONS

		// Toggle drive mode if single driver interface is active
		if (INTERFACE_SINGLEDRIVER == true && controlDriver.getRawButton(BUTTON_START) == true) {
			if (singleDriverController == 0) singleDriverController = 1; else singleDriverController = 0;
		}
		
		// Record match time
		matchTime = DriverStation.getInstance().getMatchTime();

		// Run action queues
		runQueues(actionQueues);
		
		// Dashboard dump
		SmartDashboard.putNumber("ControllerID",singleDriverController);
		SmartDashboard.putNumber("Encoder1:", encoderAngle[0].get());
		SmartDashboard.putNumber("Encoder2:", encoderAngle[1].get());
		SmartDashboard.putNumber("Encoder3:", encoderAngle[2].get());
		SmartDashboard.putNumber("Encoder4:", encoderAngle[3].get());
		SmartDashboard.putNumber("YawAxis",ahrs.getYaw());
		SmartDashboard.putBoolean("DriverOriented",driverOriented);
		SmartDashboard.putNumber("MatchTime",matchTime);
		
		// End UNIVERSAL FUNCTIONS
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		if (PIDdrive[0].isEnabled()) {
			setAllPIDControllers(PIDdrive, false);
		}
		
		SmartDashboard.putNumber("Joystick y axis", controlDriver.getRawAxis(1));
		
		SmartDashboard.putNumber("Encoder1:", encoderAngle[0].get());
		SmartDashboard.putNumber("Encoder2:", encoderAngle[1].get());
		SmartDashboard.putNumber("Encoder3:", encoderAngle[2].get());
		SmartDashboard.putNumber("Encoder4:", encoderAngle[3].get());
		
		readjust();
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
		if (controlDriver.getRawButton(5)) motorAngle[wheelTune].set(0.3);
		else if (controlDriver.getRawButton(6)) motorAngle[wheelTune].set(-0.3); else motorAngle[wheelTune].set(0);
		
		// Spin wheels
		if (controlDriver.getRawAxis(2) > .09) motorDrive[wheelTune].set(controlDriver.getRawAxis(2) / 2);
		else if (controlDriver.getRawAxis(3) > .09) motorDrive[wheelTune].set(-controlDriver.getRawAxis(3) / 2);
		else motorDrive[wheelTune].set(0);
	}

	/**
	 * Drives the robot. If emergency tank mode is enabled, then the swerve wheels will behave as two pairs of tank wheels.
	 */
	public void drive() {
		if (!emergencyTank) {
			if (!emergencyReadjust && (!controlWorking.getRawButton(BUTTON_LSTICK) && !controlWorking.getRawButton(BUTTON_RSTICK))) {
				// Drive the robot, will adjust driverOriented based on toggled input
				jFwd = -controlWorking.getRawAxis(1);if (Math.abs(jFwd) < CONTROL_DEADZONE) jFwd = 0;
				if (!controlWorking.getRawButton(BUTTON_RB) && controlWorking.getRawAxis(2) < .7) jFwd *= CONTROL_SPEEDREDUCTION;
				if (controlWorking.getRawAxis(2) >= .7) jFwd /= CONTROL_SPEEDREDUCTION_PRECISION;

				jStr = controlWorking.getRawAxis(0);if (Math.abs(jStr) < CONTROL_DEADZONE) jStr = 0;
				if (!controlWorking.getRawButton(BUTTON_RB) && controlWorking.getRawAxis(2) < .7) jStr *= CONTROL_SPEEDREDUCTION;
				if (controlWorking.getRawAxis(2) >= .7) jStr /= CONTROL_SPEEDREDUCTION_PRECISION;

				jRcw = controlWorking.getRawAxis(4);if (Math.abs(jRcw) < CONTROL_DEADZONE) jRcw = 0;
				if (!controlWorking.getRawButton(BUTTON_RB) && controlWorking.getRawAxis(2) < .7) jRcw *= CONTROL_SPEEDREDUCTION;
				if (controlWorking.getRawAxis(2) >= .7) jRcw /= CONTROL_SPEEDREDUCTION_PRECISION;

				if (reverseRotate) {jRcw=-jRcw;}
				//if (jFwd != 0 && jStr != 0 && jRcw != 0) swerve(jFwd,jStr,jRcw,driverOriented); // the conditional here made the wheels NOT turn to 0,0,0
				swerve(jFwd,jStr,jRcw,driverOriented);
			}
		} else {
			setAllPIDSetpoints(PIDdrive, 0);
			resetAllWheels();
			motorDrive[0].set(controlWorking.getRawAxis(5));motorDrive[3].set(controlWorking.getRawAxis(5));
			motorDrive[2].set(controlWorking.getRawAxis(1));motorDrive[1].set(controlWorking.getRawAxis(1));
		}
	}

	/**
	 * This adjusts the angle of the wheels and sets their speed based on joystick/autonomous input.
	 * 
	 * @param FWD The desired forward speed of the robot
	 * @param STR The desired strafing speed of the robot
	 * @param RCW The desired rotation speed of the robot
	 */
	public static void swerve(double FWD,double STR,double RCW,boolean driverOriented) {
		if (driverOriented) {
			rads = ahrs.getYaw() * Math.PI/180;
			temp = FWD*Math.cos(rads) + STR*Math.sin(rads);
			STR = -FWD*Math.sin(rads) + STR*Math.cos(rads);
			FWD = temp;
		}

		a = STR - RCW * (ROBOT_LENGTH / ROBOT_R);
		b = STR + RCW * (ROBOT_LENGTH / ROBOT_R);
		c = FWD - RCW * (ROBOT_WIDTH / ROBOT_R);
		d = FWD + RCW * (ROBOT_WIDTH / ROBOT_R);

		//1..4: front_right, front_left, rear_left, rear_right

		wheelSpeed1 = Math.sqrt(Math.pow(b,2)+Math.pow(c,2));
		wheelSpeed2 = Math.sqrt(Math.pow(b,2)+Math.pow(d,2));
		wheelSpeed3 = Math.sqrt(Math.pow(a,2)+Math.pow(d,2));
		wheelSpeed4 = Math.sqrt(Math.pow(a,2)+Math.pow(c,2));

		encoderSetpointA = wheel[0].calculateWheelAngle(b,c);
		PIDdrive[0].setSetpoint(encoderSetpointA);SmartDashboard.putNumber("Enc. A setpoint", encoderSetpointA);
		
		encoderSetpointB = wheel[1].calculateWheelAngle(b,d);
		PIDdrive[1].setSetpoint(encoderSetpointB);SmartDashboard.putNumber("Enc. B setpoint", encoderSetpointB);
		
		encoderSetpointC = wheel[2].calculateWheelAngle(a,d);
		PIDdrive[2].setSetpoint(encoderSetpointC);SmartDashboard.putNumber("Enc. C setpoint", encoderSetpointC);
		
		encoderSetpointD = wheel[3].calculateWheelAngle(a,c);
		PIDdrive[3].setSetpoint(encoderSetpointD);SmartDashboard.putNumber("Enc. D setpoint", encoderSetpointD);

		max=wheelSpeed1; if(wheelSpeed2>max)max=wheelSpeed2; if(wheelSpeed3>max)max=wheelSpeed3; if(wheelSpeed4>max)max=wheelSpeed4;
		if (max > 1) {wheelSpeed1/=max; wheelSpeed2/=max; wheelSpeed3/=max; wheelSpeed4/=max;}
		
		wheelSpeed1 *= wheel[0].getFlip();
		wheelSpeed2 *= wheel[1].getFlip();
		wheelSpeed3 *= wheel[2].getFlip();
		wheelSpeed4 *= wheel[3].getFlip();
		
		//Move[2].set(testStick.getRawAxis(1));
		
		if (wheelSpeedTimer.get()>0.1) {
			if (wheelSpeed1 - wheelSpeedActual1 > 0.1) {wheelSpeedActual1 += 0.1;} else if (wheelSpeed1 - wheelSpeedActual1 < -0.1) {wheelSpeedActual1 -= 0.1;} else {wheelSpeedActual1 = wheelSpeed1;}
			if (wheelSpeed2 - wheelSpeedActual2 > 0.1) {wheelSpeedActual2 += 0.1;} else if (wheelSpeed2 - wheelSpeedActual2 < -0.1) {wheelSpeedActual2 -= 0.1;} else {wheelSpeedActual2 = wheelSpeed2;}
			if (wheelSpeed3 - wheelSpeedActual3 > 0.1) {wheelSpeedActual3 += 0.1;} else if (wheelSpeed3 - wheelSpeedActual3 < -0.1) {wheelSpeedActual3 -= 0.1;} else {wheelSpeedActual3 = wheelSpeed3;}
			if (wheelSpeed4 - wheelSpeedActual4 > 0.1) {wheelSpeedActual4 += 0.1;} else if (wheelSpeed4 - wheelSpeedActual4 < -0.1) {wheelSpeedActual4 -= 0.1;} else {wheelSpeedActual4 = wheelSpeed4;}
			wheelSpeedTimer.reset();
		}
		//Move[0].set(wsActual1);Move[1].set(wsActual2);Move[2].set(wsActual3);Move[3].set(wsActual4);
		
		motorDrive[0].set(wheelSpeed1);motorDrive[1].set(wheelSpeed2);motorDrive[2].set(wheelSpeed3);motorDrive[3].set(wheelSpeed4);
	}

	/**
	 * Resets all of the SwerveWheel objects, putting them on a clean slate
	 * (eliminates flipped orientations, stacked setpoints, etc.)
	 */
	public void resetAllWheels() {
		for (int i = 0; i <= 3; i++) {
			wheel[i].reset();
		}
	}

	/**
	 * Sets all drive wheels to a single value. This is good for turning all the motors off.
	 * @param val is the value to set all the wheels to
	 */
	public void setAllWheels(double val) {
		for (int i = 0; i <= 3; i ++) {
			motorDrive[i].set(val * wheel[i].getFlip());
		}
	}
	
	/**
	 * Enables or disables a given array of four PIDController objects.
	 * @param pids The array of PID Controllers to set
	 * @param enabled True to enable, false to disable
	 */
	public void setAllPIDControllers(PIDController[] pids, boolean enabled) {
		for (int i=0;i<=3;i++) {
			pids[i].setEnabled(enabled);
		}
	}

	/**
	 * Sets the setpoints for an array of four PIDController objects.
	 * @param pids The array of PID Controllers to set
	 * @param setpoint The setpoint
	 */
	public void setAllPIDSetpoints(PIDController[] pids, double setpoint) {
		for (int i=0;i<=3;i++) {
			pids[i].setSetpoint(setpoint);
		}
	}

	/**
	 * Returns a value based on sensor inputs.
	 * @param p - the proportional constant
	 * @param currentSensor - whatever your current sensor value is
	 * @param desiredSensor - whatever you want the sensor to become after change
	 */
	public static double proportionalLoop(double p, double currentSensor, double desiredSensor) {
		return p * (currentSensor - desiredSensor);
	}

	/**
	 * Tell all of the action queues to run if they are enabled.
	 * @param queues[] the array of queues to iterate through
	 */
	public void runQueues(ActionQueue queues[]) {
		for (int i = 0; i < queues.length; i++) {
			if (queues[i].queueIsRunning == true) queues[i].queueRun();
		}
	}

	/**
	 * Kills all action queues in a specified array, regardless of whether they're enabled or not
	 * @param queues[] the array of queues to kill
	 */
	public void killQueues(ActionQueue queues[]) {
		for (int i = 0; i < queues.length; i++) {
			queues[i].queueStop();
		}
	}

	/**
	 * The queue action for preparing a turn. This is functionally similar to the queueSwerve command
	 * @param timeEnd the designated time for the command to end
	 * @param param1 the first parameter, the fwd input
	 * @param param2 the second parameter, the str input
	 */
	public static void queuePrepare_Turn(int timeEnd, double param1, double param2) {
		swerve(param1,param2,0,false);
	}

	/**
	 * The queue action for swerving in its raw form. This is completed relative to the ROBOT.
	 * @param timeEnd the designated time for the command to end
	 * @param param1 the first parameter, FWD
	 * @param param2 the second parameter, STR
	 * @param param3 the third parameter, RCW
	 */
	public static void queueSwerve(int timeEnd, double param1, double param2, double param3) {
		swerve(param1,param2,param3,false);
	}
}
