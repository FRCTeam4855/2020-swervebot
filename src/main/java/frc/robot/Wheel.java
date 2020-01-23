// FRC TEAM 4855 ROBOT CODE
// 2020 GAME INFINITE RECHARGE

// Swerve-bot code: Wheel class

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;

/**
 * A single, physical wheel. It contains attributes that aid in its calculation of angles and its motor controllers.
 */
public class Wheel {
	public TalonSRX motorAngle;														// the motor controller that drives the angular motor of the wheel
	public VictorSPX motorDrive;														// the motor controller that drives the rotational motor of the wheel
	public AnalogInput zeroSensor;													// the magnetic sensor mounted at the wheel's 0 degree mark
	public PIDController PID;														// the specific PID controller for this wheel's angular motion

	final private double ETD = Robot.ENC_TO_DEG; 									// encoder to degrees
	private int myID;																// unique numeric id that describes the order of this wheel in relation to the others
	private double setpoint = 0;													// the setpoint of the angular encoder
	public boolean on = false;														// whether or not the wheel is "active" and responding to control
	private double angleCalc = 0, flip = 0, flipCorrection = 0, anglePrevious = 0;	// calculation variables for swerve angles
	private boolean lockFlip = false;												// strictly for the 2018 auto program
	private boolean settingToZero = false;											// flag for whether or not the wheel is currently trying to find its zero
	private boolean settingToZeroReset = false;										// flag for whether or not the wheel should reset itself when zero is reached
	private int zeroErrorCorrectTimer = -1;											// the timer for the zeroing process correcting itself
	private final int ZERO_CORRECT_TIME = 12;										// the amount of time the robot uses correcting the natural drift of the zeroing process
	private boolean faulty = false;													// whether or not this wheel instance has been flagged for faulty rotational behavior
	private int faultySetpointTimer = -1;											// counts how long a disparity has existed between the PID setpoint and the encoder 
	private boolean lockAtZero = false;												// whether the wheels should be locked at zero using the analog input or not

	private double kP = 0.0089;		// formerly .0071
	private double kI = 0.00000008;	// formerly .00008
	private double kD = 0.0001;		// formerly .000056

	/**
	 * Creates a new wheel instance. There should only be four of these
	 * @param a the motor controller that controls the angle of the wheel
	 * @param d the motor control that controls the direction of the wheel
	 */
	public Wheel(TalonSRX a, VictorSPX d, AnalogInput i, int id) {
		motorAngle = a;
		motorDrive = d;
		zeroSensor = i;
		myID = id;
		PID = new PIDController(kP, kI, kD);	// uses encoderAngle to set motorAngle
		PID.setTolerance(20);
	}
	
	/**
	 * Takes two measurement arguments provided by Robot's swerve() method and spits out an angle measurement
	 * @param dL1 first calculation
	 * @param dL2 second calculation
	 * @return double for the calculated angle for the wheel to turn to
	 */
	public double calculateWheelAngle(double dL1, double dL2) {
		
		angleCalc = -(Math.atan2(dL1, dL2) * 180 / Math.PI) * ETD;	// DON'T TOUCH THIS
		// The below line makes the swerves generally efficient, however it makes completely straight FWD joystick inputs not turn the wheels at all
		if (angleCalc == 0) angleCalc = anglePrevious;	// if the angled is 0, set the angle to whatever the last one was
		
		angleCalc += flipCorrection + flip;
		
		
		// If the wheel needs to turn more than 180 degrees to reach the target, flip input
		if (Math.abs(getEncoderPosition() - angleCalc) > 90 * ETD && Math.abs(getEncoderPosition() - angleCalc) < 270 * ETD) {
			angleCalc -= flip;
			if (flip == 0)
				flip = 180 * ETD;
			else
				flip = 0;
			angleCalc += flip;
		}

		// This is just for autonomous rotation to angle
		if (lockFlip) {
			if (flip != 0) {
				angleCalc -= flip;
				flip = 0;
			}
		}

		// Correct encoder setpoints if they've already flipped a bunch of times

		if (anglePrevious - angleCalc > 185 * ETD) {
			flipCorrection += 360 * ETD;
			angleCalc += 360 * ETD;
		} // For magnetic encoders, USE 412 (ABS 4048)
		if (anglePrevious - angleCalc < -185 * ETD) {
			flipCorrection -= 360 * ETD;
			angleCalc -= 360 * ETD;
		}
		if (getEncoderPosition() - angleCalc > 380 * ETD) {
			flipCorrection += 360 * ETD;
			angleCalc += 360 * ETD;
		}
		if (getEncoderPosition() - angleCalc < -380 * ETD) {
			flipCorrection -= 360 * ETD;
			angleCalc -= 360 * ETD;
		}
		anglePrevious = angleCalc;
		return angleCalc;
	}
	
	public int getFlip() {
		if (flip == 0) return 1; else return -1;
	}
	
	public void lockFlip(boolean f) {
		lockFlip = f;
	}
	// enable.hotdogstand < 0 ((( ><  Cody did this- cody
	/**
	 * Resets the calculation properties of the wheel. Also tells the wheel that it has not performed any flips
	 */
	public void reset() {
		angleCalc = 0;
		flip = 0;
		anglePrevious = 0;
		flipCorrection = 0;
	}

	/**
	 * Returns true if the analog sensor is being tripped and false if not
	 * @return boolean for whether or not analog sensor is being tripped (wheel is pointed to 0 degrees)
	 */
	public boolean isZero() {
		//SensorCollection sensors = motorAngle.getSensorCollection();
		return (zeroSensor.getVoltage() < 1);
	}

	/**
	 * Returns the raw analog sensor value.
	 */
	public double getRawAnalog() {
		//SensorCollection sensors = motorAngle.getSensorCollection();
		//return (sensors.getAnalogInRaw());
		return zeroSensor.getVoltage();
	}

	/**
	 * Instructs the wheel to begin rotating until its zero is found
	 * @param resetWheel true if you would like the wheel to reset its rotation count and encoder position when zero is reached
	 */
	public void setToZero(boolean resetWheel) {
		settingToZero = true;
		settingToZeroReset = resetWheel;
	}

	/**
	 * Prematurely ends the routine to set the wheel to zero
	 */
	public void endSetToZero() {
		settingToZero = false;
	}

	/**
	 * Returns true if the wheel has deemed its behavior faulty
	 * @return boolean for whether or not this wheel is faulty or not
	 */
	public boolean getFaulty() {
		return faulty;
	}

	/**
	 * Clears an existing fault. If the wheel has decided that it is faulty, this is the only way to turn that off besides using the clean slate startup.
	 */
	public void clearFault() {
		faulty = false;
	}
	
	/**
	 * Locks the wheel at 0. If the wheel is flagged as faulty, the setpoint will be foregone and the analog sensor will be
	 * the sole source of information on whether or not the wheel has stayed.
	 */
	public void lock() {
		lockAtZero = true;
	}

	/**
	 * Unlocks the wheel.
	 */
	public void unlock() {
		lockAtZero = false;
	}

	/**
	 * Returns the setpoint of the angular motor.
	 */
	public double getSetpoint() {
		return setpoint;
	}

	/**
	 * Returns the position of the encoder.
	 */
	public double getEncoderPosition() {
		SensorCollection sensors = motorAngle.getSensorCollection();
		return sensors.getQuadraturePosition();
	}

	/**
	 * Sets the setpoint of the angular motor.
	 * @param s The new setpoint in encoder units
	 */
	public void setSetpoint(int s) {
		setpoint = s;
	}

	/**
	 * Resets the encoder position to 0.
	 */
	public void resetEncoderPosition() {
		SensorCollection sensors = motorAngle.getSensorCollection();
		sensors.setQuadraturePosition(0, 0);
	}

	/**
	 * Turns the wheel on, making the PIDController's calculate() method run.
	 */
	public void turnOn() {
		on = true;
	}

	/**
	 * Turns the wheel on and clean slates all of its attributes.
	 */
	public void turnOnCleanSlate() {
		PID.reset();
		setSetpoint(0);
		resetEncoderPosition();
		unlock();
		endSetToZero();
		reset();
		clearFault();
		motorDrive.set(ControlMode.PercentOutput, 0);
		motorAngle.set(ControlMode.PercentOutput, 0);
		turnOn();
	}

	/**
	 * Turns the wheel off, preventing encoder setpoints from moving the wheel.
	 */
	public void turnOff() {
		on = false;
	}

	/**
	 * This function should run constantly while the robot is enabled. Spins the wheel when setToZero() has been called and
	 * runs checks to ensure the wheel is not exhibiting faulty behavior. This function also outputs diagnostic information to
	 * the SmartDashboard.
	 */
	public void process() {
		// Turn wheel to zero using the analog magnetic sensor. This does not use encoders and should only be attempted in an emergency
		if (settingToZero) {
			if (isZero() || zeroErrorCorrectTimer != -1) {
				// Correcting for natural, consistent error
				if (zeroErrorCorrectTimer == -1) {
					zeroErrorCorrectTimer = ZERO_CORRECT_TIME;
				}
				zeroErrorCorrectTimer --;
				int sign = -1;
				if (myID > 1) sign = 1;
				motorAngle.set(ControlMode.PercentOutput, .7 * sign);

				// Natural error correction has ended, end the zeroing process
				if (zeroErrorCorrectTimer == 0) {
					settingToZero = false;
					motorAngle.set(ControlMode.PercentOutput, 0);
					zeroErrorCorrectTimer = -1;
					if (settingToZeroReset) {
						reset();
						resetEncoderPosition();
					}
				}
			} else if (zeroErrorCorrectTimer == -1) {
				int sign = 1;
				if (myID > 1) sign = -1;
				motorAngle.set(ControlMode.PercentOutput, .9 * sign);
			}
		}

		// Check for faulty status, encoder should read a constant 0 if it is failing
		if (Math.abs(getSetpoint()) > 20 && getEncoderPosition() == 0) {
			// Begin counting
			faultySetpointTimer ++;
			if (faultySetpointTimer > 120) {
				// Apparent irresponsiveness for greater than 2 seconds, wheel is likely faulty
				faulty = true;
			}
		} else {
			faultySetpointTimer = -1;
		}
		
		// Make sure the wheel is locked at zero
		if (lockAtZero) {
			if (faulty && !isZero()) {
				setToZero(true);
			}
			setSetpoint(0);
		}

		// Turn the wheels based on encoder input
		if (on && !settingToZero) {
			double set = setpoint / 4;
			double calc = PID.calculate(getEncoderPosition() / 4, set);
			calc = MathUtil.clamp(calc, -1, 1);
			motorAngle.set(ControlMode.PercentOutput, calc);
		} else if (!settingToZero) {
			motorDrive.set(ControlMode.PercentOutput, 0);
			motorAngle.set(ControlMode.PercentOutput, 0);
		}

		
		/*
		kP = SmartDashboard.getNumber("P", kP);
		kI = SmartDashboard.getNumber("I", kI);
		kD = SmartDashboard.getNumber("D", kD);

		setpoint = SmartDashboard.getNumber("Wheel " + myID + "Setpoint", 0);

		SmartDashboard.putNumber("P", kP);
		SmartDashboard.putNumber("I", kI);
		SmartDashboard.putNumber("D", kD);

		// PID tuning
		PID.setP(kP);
		PID.setI(kI);
		PID.setD(kD);*/

		// Put wheel stats on the SmartDashboard
		
		SmartDashboard.putNumber("Wheel " + myID + "Encoder", getEncoderPosition());
		SmartDashboard.putNumber("Wheel " + myID + "Setpoint", getSetpoint());
		SmartDashboard.putBoolean("Wheel " + myID + "Fault", getFaulty());
		SmartDashboard.putBoolean("Wheel " + myID + "Zeroed", isZero());
	}

	/**
	 * Turns the wheel to the setpoint. Should be running constantly.
	 */
	public void setTurn() {
		if (on) motorAngle.set(ControlMode.PercentOutput, MathUtil.clamp(PID.calculate(getEncoderPosition(), getSetpoint()), -1, 1));
	}
}

