// FRC TEAM 4855 ROBOT CODE
// 2020 GAME INFINITE RECHARGE

// Swerve-bot code: Utility

package frc.robot;

/**
 * This class contains utility features that are used universally and don't belong in the Robot class.
 * This class should never be constructed, as its methods and attributes are all static and unchanging.
 */
public class Utility {
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
	 * Resets all of the SwerveWheel objects, putting them on a clean slate
	 * (eliminates flipped orientations, stacked setpoints, etc.). This doesn't
	 * actually reset their encoder values; for that, use resetAllEncoders()
	 */
	public static void resetAllWheels(Wheel[] wheel) {
		for (Wheel w : wheel) {
			w.reset();
		}
	}
	
	/**
	 * Calibrates each wheel using its analog sensor.
	 */
	public static void zeroAllWheelsWithAnalog(Wheel[] wheel) {
		for (Wheel w : wheel) {
			w.setToZero(true);
		}
	}

	/**
	 * Turns on and off each of the robot's Wheel objects.
	 * @param enabled True to enable, false to disable
	 * @param wheel an array of Wheel objects
	 */
	public static void powerAllWheels(boolean enabled, Wheel[] wheel) {
		for (Wheel w : wheel) {
			if (enabled) w.turnOn(); else w.turnOff();
		}
	}

	/**
	 * Turns on each wheel with a clean slate. The encoder is reset, the math is reset, and all motors are shut off before starting up.
	 * @param wheel an array of Wheel objects
	 */
	public static void cleanSlateAllWheels(Wheel[] wheel) {
		for (Wheel w : wheel) {
			w.turnOnCleanSlate();
		}
	}

	/**
	 * Sets the setpoints for every Wheel object in the wheel array.
	 * @param setpoint The setpoint to assign to every wheel
	 * @param wheel an array of wheels
	 */
	public static void setAllPIDSetpoints(int setpoint, Wheel[] wheel) {
		for (Wheel w : wheel) {
			w.setSetpoint(setpoint);
		}
	}

	/**
	 * Resets the encoders of all the wheel objects, setting all their counts to 0.
	 */
	public static void resetAllEncoders(Wheel[] wheel) {
		for (Wheel w : wheel) {
			w.resetEncoderPosition();
		}
	}
}