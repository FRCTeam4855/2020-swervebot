// FRC TEAM 4855 ROBOT CODE
// 2020 GAME INFINITE RECHARGE

// Swerve-bot code: Wheel class

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;

/**
 * A single, physical wheel. It contains attributes that aid in its calculation of angles and its motor controllers.
 */
public class Wheel {
	public Encoder encoderAngle;													// the encoder object for this instance of the wheel
	public Spark motorAngle;														// the motor controller that drives the angular motor of the wheel
	public Spark motorDrive;														// the motor controller that drives the rotational motor of the wheel
	private AnalogInput zeroSensor;													// the magnetic sensor mounted at the wheel's 0 degree mark
	public PIDController PID;														// the specific PID controller for this wheel's angular motion

	final private double ETD = Robot.ENC_TO_DEG; 									// encoder to degrees
	private double angleCalc = 0, flip = 0, flipCorrection = 0, anglePrevious = 0;	// calculation variables for swerve angles
	private boolean lockFlip = false;												// strictly for the 2018 auto program
	private boolean settingToZero = false;											// flag for whether or not the wheel is currently trying to find its zero
	private boolean settingToZeroReset = false;										// flag for whether or not the wheel should reset itself when zero is reached
	private boolean faulty = false;													// whether or not this wheel instance has been flagged for faulty rotational behavior
	private int faultySetpointTimer = -1;											// counts how long a disparity has existed between the PID setpoint and the encoder 
	private boolean lockAtZero = false;												// whether the wheels should be locked at zero using the analog input or not

	/**
	 * Creates a new wheel instance. There should only be four of these
	 * @param e an encoder instance for this module
	 * @param a the motor controller that controls the angle of the wheel
	 * @param d the motor control that controls the direction of the wheel
	 */
	public Wheel(Encoder e, Spark a, Spark d, AnalogInput i) {
		encoderAngle = e;
		motorAngle = a;
		motorDrive = d;
		zeroSensor = i;
		PID = new PIDController(0.035, 0, 0.01, encoderAngle, motorAngle);
		PID.setOutputRange(-1, 1);
	}
	
	/**
	 * Takes two measurement arguments provided by Robot's swerve() method and spits out an angle measurement
	 * @param dL1 first calculation
	 * @param dL2 second calculation
	 * @return double for the calculated angle for the wheel to turn to
	 */
	public double calculateWheelAngle(double dL1, double dL2) {
		
		angleCalc = -(Math.atan2(dL1, dL2) * 180 / Math.PI) * ETD;	// DON'T TOUCH THIS
		//if (angleCalc == 0) angleCalc = anglePrevious;	// if the angled is 0, set the angle to whatever the last one was
		
		angleCalc += flipCorrection + flip;
		
		
		// If the wheel needs to turn more than 180 degrees to reach the target, flip input
		if (Math.abs(encoderAngle.get() - angleCalc) > 90 * ETD && Math.abs(encoderAngle.get() - angleCalc) < 270 * ETD) {
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
		if (encoderAngle.get() - angleCalc > 380 * ETD) {
			flipCorrection += 360 * ETD;
			angleCalc += 360 * ETD;
		}
		if (encoderAngle.get() - angleCalc < -380 * ETD) {
			flipCorrection -= 360 * ETD;
			angleCalc -= 360 * ETD;
		}
		anglePrevious = angleCalc;
		return angleCalc;
	}
	
	public int getFlip() {
		if (flip==0) return 1; else return -1;
	}
	
	public void lockFlip(boolean f) {
		lockFlip = f;
	}

	/**
	 * Resets the calculation properties of the wheel. Also tells the wheel that it has not performed any flips
	 */
	public void reset() {
		angleCalc = 0;flip = 0;anglePrevious = 0;flipCorrection = 0;
	}

	/**
	 * Returns true if the analog sensor is being tripped and false if not
	 * @return boolean for whether or not analog sensor is being tripped (wheel is pointed to 0 degrees)
	 */
	public boolean isZero() {
		return (zeroSensor.getValue() < 140);
	}

	/**
	 * Instructs the wheel to begin rotating until its zero is found
	 * @param resetWheel true if you would like the wheel to reset itself completely when zero is reached
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
	 * This function should run constantly while the robot is enabled. Spins the wheel when setToZero() has been called and
	 * runs checks to ensure the wheel is not exhibiting faulty behavior. This function also outputs diagnostic information to
	 * the SmartDashboard.
	 */
	public void process() {
		// Turn wheel to zero
		if (settingToZero) {
			if (isZero()) {
				settingToZero = false;
				motorAngle.set(0);
				if (settingToZeroReset) {
					reset();
					encoderAngle.reset();
				}
			} else {
				motorAngle.set(.6);
			}
		}

		// Check for faulty status, encoder should read a constant 0 if it is failing
		if (Math.abs(PID.getSetpoint()) > 20 && encoderAngle.get() == 0) {
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
			PID.setSetpoint(0);
		}

		// TODO SmartDashboard output
	}
}

