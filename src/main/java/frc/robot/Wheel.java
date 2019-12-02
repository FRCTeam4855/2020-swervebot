// FRC TEAM 4855 ROBOT CODE
// 2020 GAME INFINITE RECHARGE

// Swerve-bot code: Wheel class

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;

public class Wheel {
	private double angleCalc = 0, flip = 0, flipCorrection = 0, anglePrevious = 0;
	boolean lockFlip = false;	// strictly for the 2018 auto program
	final double ETD = 1.158333; //ENCODER TO DEGREES
	Encoder encoder;
	
	public Wheel(Encoder e) {
		encoder = e;
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
		if (Math.abs(encoder.get() - angleCalc) > 90 * ETD && Math.abs(encoder.get() - angleCalc) < 270 * ETD) {
			angleCalc -= flip;
			if (flip == 0) flip = 180 * ETD; else flip = 0;
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
		
		if (anglePrevious - angleCalc > 185*ETD) {flipCorrection += 360*ETD;angleCalc += 360*ETD;} //For magnetic encoders, USE 412 (ABS 4048)
		if (anglePrevious - angleCalc < -185*ETD) {flipCorrection -= 360*ETD;angleCalc -= 360*ETD;}
		if (encoder.get() - angleCalc > 380*ETD) {flipCorrection += 360*ETD;angleCalc += 360*ETD;}
		if (encoder.get() - angleCalc < -380*ETD) {flipCorrection -= 360*ETD;angleCalc -= 360*ETD;}
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
}

