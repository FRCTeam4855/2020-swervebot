// FRC TEAM 4855 ROBOT CODE
// 2020 GAME INFINITE RECHARGE

// Swerve-bot code: Wheel class

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;

/**
 * A single, physical wheel. It contains attributes that aid in its calculation of angles and its motor controllers.
 */
public class Wheel {
	private double angleCalc = 0, flip = 0, flipCorrection = 0, anglePrevious = 0;	// calculation variables for swerve angles
	private boolean lockFlip = false;												// strictly for the 2018 auto program
	public Encoder encoderAngle;													// the encoder object for this instance of the wheel
	public TalonSRX motorAngle;														// the motor controller that drives the angular motor of the wheel
	public VictorSPX motorDrive;													// the motor controller that drives the rotational motor of the wheel
	private double setpoint;														// angular encoder setpoint
	public PIDController PID;														// PID controller for this wheel
	final private double ETD = Robot.ENC_TO_DEG; 									// encoder to degrees
	public boolean on = false;														// whether the wheel is supposed to calculate or not

	/**
	 * Creates a new wheel instance. There should only be four of these
	 * @param a the motor controller that controls the angle of the wheel
	 * @param d the motor control that controls the direction of the wheel
	 */
	public Wheel(TalonSRX a, VictorSPX d) {
		motorAngle = a;
		motorDrive = d;
		PID = new PIDController(0.035, 0.0, 0.01);
	}
	
	/**
	 * Returns the position of the encoder.
	 */
	public int getEncoderPosition() {
		SensorCollection sensors = motorAngle.getSensorCollection();
		return sensors.getQuadraturePosition();
	}

	/**
	 * Sets the position of the encoder. Usually only used for resetting the encoder back to 0.
	 * @param s
	 */
	public void setEncoderPosition(int s) {
		SensorCollection sensors = motorAngle.getSensorCollection();
		sensors.setQuadraturePosition(s, 0);
	}

	/**
	 * Gets the setpoint of the encoder.
	 */
	public double getSetpoint() {
		return setpoint;
	}

	/**
	 * Sets the setpoint for the angular motor.
	 * @param s the setpoint to set it to
	 */
	public void setSetpoint(double s) {
		setpoint = s;
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
	
	public void reset() {
		angleCalc = 0;flip = 0;anglePrevious = 0;flipCorrection = 0;
	}

	/**
	 * Turns the wheel to the setpoint. Should be running constantly.
	 */
	public void setTurn() {
		if (on) motorAngle.set(ControlMode.PercentOutput, MathUtil.clamp(PID.calculate(getEncoderPosition(), getSetpoint()), -1, 1));
	}
}

