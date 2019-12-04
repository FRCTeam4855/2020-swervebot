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
}