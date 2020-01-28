// FRC TEAM 4855 ROBOT CODE
// 2020 GAME INFINITE RECHARGE

// Swerve-bot code: Limelight class

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Pulls Limelight data from NetworkTables.
 */
public class Limelight {
	boolean lampOn = true;

	NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");							// creates the limelight table
	NetworkTableEntry x = limelightTable.getEntry("tx");															// the x offset from the crosshairs
	NetworkTableEntry y = limelightTable.getEntry("ty");															// the y offset from the crosshairs
	NetworkTableEntry area = limelightTable.getEntry("ta");															// the area (0-100) of the object
	NetworkTableEntry found = limelightTable.getEntry("tv");														// 1 if object is tracking, 0 if not
	NetworkTableEntry boxWidth = limelightTable.getEntry("thor");													// 0-320 width of the box
	NetworkTableEntry boxHeight = limelightTable.getEntry("tvert");													// 0-320 height of the box

	NetworkTableEntry boxShort = limelightTable.getEntry("tshort");													// 0-320 value of shortest side
	NetworkTableEntry boxLong = limelightTable.getEntry("tlong");													// 0-320 value of longest side

	NetworkTableEntry ledMode = limelightTable.getEntry("ledMode");													// 0 for on, 1 for off
	NetworkTableEntry camMode = limelightTable.getEntry("camMode");
    
    public Limelight() {
	
	}

	public void turnOnLamp() {
		ledMode.setNumber(0);
		camMode.setNumber(0);
		lampOn = true;
	}

	public void turnOffLamp() {
		ledMode.setNumber(1);
		camMode.setNumber(1);
		lampOn = false;
	}

	public void toggleLamp() {
		if (lampOn) {
			turnOffLamp();
		} else turnOnLamp();
	}

	/**
	 * Returns the x value of the target relative to Limelight's crosshairs
	 * @return a double in Limelight units the displacement between the target and the crosshairs
	 */
	public double getTargetX() {
		return x.getDouble(0);
	}

	/**
	 * Returns whether or not Limelight sees a target on screen
	 * @return true or false if a target exists
	 */
	public boolean getTargetExists() {
		return found.getDouble(0) == 1;
	}
}