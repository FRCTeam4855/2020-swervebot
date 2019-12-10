// FRC TEAM 4855 ROBOT CODE
// 2020 GAME INFINITE RECHARGE

// Swerve-bot code: Wheel class

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;

/**
 * A single, physical wheel. It contains attributes that aid in its calculation of angles and its motor controllers.
 */
public class Wheel {
	private double angleCalc = 0, flip = 0, flipCorrection = 0, anglePrevious = 0;	// calculation variables for swerve angles
	private boolean lockFlip = false;												// strictly for the 2018 auto program
	public Encoder encoderAngle;													// the encoder object for this instance of the wheel
	public Spark motorAngle;														// the motor controller that drives the angular motor of the wheel
	public Spark motorDrive;														// the motor controller that drives the rotational motor of the wheel
	private AnalogInput zeroSensor;													// the magnetic sensor mounted at the wheel's 0 degree mark
	final private double ETD = Robot.ENC_TO_DEG; 									// encoder to degrees
	private boolean faulty = false;													// whether or not this wheel instance has been flagged for faulty rotational behavior

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
	}
	
	/**
	 * This function takes
	 * oh bubbers looks like this javadoc never got finished... this function is a mystery...
	 * @param dL1
	 * @param dL2
	 * @return
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
	 * Returns true if the analog sensor is being tripped and false if not
	 */
	public boolean isZero() {
		return (zeroSensor.getValue() < 140);
	}

	public void reset() {
		angleCalc = 0;flip = 0;anglePrevious = 0;flipCorrection = 0;
	}
}

