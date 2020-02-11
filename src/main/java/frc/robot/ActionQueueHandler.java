// FRC TEAM 4855 ROBOT CODE
// 2020 GAME INFINITE RECHARGE

// Swerve-bot code: Action Queue Handler class

package frc.robot;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpiutil.math.MathUtil;

/**
 * Runs all ActionQueue instances and is responsible for holding methods that
 * run the robot and holding onto all ActionQueue objects.
 */
public class ActionQueueHandler {
    private ActionQueue[] queues;
	private static PIDController PIDRotate = new PIDController(.0077, 0, 0.0007);
	private static PIDController PIDLimelightXRot = new PIDController(.0285, 0, 0.0014);

	/**
	 * Constructs an ActionQueueHandler. It is responsible for managing the running of every
	 * queue and sending commands to Robot. When constructed, references to each of the robot's
	 * primary components are created
	 * @param aq an array of ActionQueue objects. This should include every Action Queue that you want
	 * to run on the robot.
	 */
    public ActionQueueHandler(ActionQueue[] aq) {
		queues = aq;
    }

    /**
     * Gets a queue of a certain index
     * @param index the index of the desired queue
     * @return the ActionQueue object of the specified index
     */
    public ActionQueue getQueue(int index) {
        return queues[index];
    }

    /**
	 * Tell all of the action queues to run if they are enabled.
	 * @param queues[] the array of queues to iterate through
	 */
	public void runQueues() {
		for (int i = 0; i < queues.length; i++) {
			if (queues[i].queueRunning() == true) queues[i].queueRun();
		}
	}

	/**
	 * Kills all action queues in a specified array, regardless of whether they're enabled or not
	 * @param queues[] the array of queues to kill
	 */
	public void killQueues() {
		for (int i = 0; i < queues.length; i++) {
			queues[i].queueStop();
		}
	}

	/**
	 * The queue action for stopping an ActionQueue's flow of time until a specific sensor value is reached
	 * @param timeEnd the designated time for the stopage to start
	 * @param param1 the first parameter, the sensor to read from
	 * @param param2 the second parameter, the bottom value to wait for
	 * @param param3 the third parameter, the top value to wait for
	 * @return true if the sensor value has been reached and false if not
	 */
	public static boolean queueCheck_Sensor(double timeEnd, double param1, double param2, double param3) {
		switch ((int) param1) {
			case 0:
				return false;
			case 1:
				// Wait for the flywheel to be at the correct speed before proceeding through time
				// Only checked against bottom value
				boolean check = Robot.shooter.setFlywheelSpeed(Robot.shooter.getFlywheelSetpoint());
				return check;
			default:
				break;
		}
		return true;
	}

	/**
	 * The queue action for preparing a turn. This is functionally similar to the queueSwerve command
	 * @param timeEnd the designated time for the command to end
	 * @param param1 the first parameter, the fwd input
	 * @param param2 the second parameter, the str input
	 */
	public static void queuePrepare_Turn(double timeEnd, double param1, double param2) {
		Robot.swerve(param1, param2, 0, false);
	}

	/**
	 * The queue action for swerving in its raw form. This is completed relative to the ROBOT.
	 * @param timeEnd the designated time for the command to end
	 * @param param1 the first parameter, FWD
	 * @param param2 the second parameter, STR
	 * @param param3 the third parameter, RCW
	 */
	public static void queueSwerve(double timeEnd, double param1, double param2, double param3) {
		Robot.swerve(param1, param2, param3, false);
	}

	/**
	 * The queue action for driving in a straight line alongside a wall.
	 * @param timeEnd the designated time for the command to end
	 * @param param1 the first parameter, the power at which to drive
	 */
	public static void queueDrive_Straight(double timeEnd, double param1, double param2) {
		Robot.overrideFWD = param1;
		Robot.overrideSTR = param2;
	}

	/**
	 * The queue action for turning the entire robot to a certain angle. Uses a PIDController and applies the change to
	 * the RCW parameter of Robot's swerve() method.
	 * @param timeEnd the designated time for the command to end
	 * @param param1 the first parameter, the angle to rotate the robot to
	 */
	public static void queueTurn_To_Angle(double timeEnd, double param1) {
		double rotValue = PIDRotate.calculate(Robot.gyro.getYaw(), param1);	// calculate rotation input to a PID loop
		rotValue = MathUtil.clamp(rotValue, -.42, .42);						// make sure robot doesn't attempt to rotate too fast
		Robot.overrideRCW = rotValue;
	}

	/**
	 * The queue action for feeding a ball into the robot's intake system.
	 * @param timeEnd the designated time for the command to end
	 */
	public static void queueFeed_Ball(double timeEnd) {
		Robot.shooter.runFeeder(.9);
	}

	/**
	 * The queue action for running the flywheel to a specified speed.
	 * @param timeEnd the designated time for the command to end
	 * @param param1 the first parameter, the speed in RPM to set the flywheel to
	 * @param param2 the second parameter, which is either 1 or 0 for whether or not to use lidar
	 */
	public static void queueRun_Flywheel(double timeEnd, double param1, double param2) {
		if (param2 == 0) Robot.shooter.setFlywheelSpeed(param1); else Robot.shooter.setFlywheelSpeed(Robot.shooter.getVelocityFromDistance(Robot.lidar.getDistance(Lidar.Unit.INCHES)));
	}

	/**
	 * The queue action for running the intake wheels on the intake.
	 * @param timeEnd the designated time for the command to end
	 * @param param1 the first parameter, the percent input to feed to the wheels
	 */
	public static void queueRun_Intake_Wheels(double timeEnd, double param1) {
		Robot.intake.setIntakeWheels(param1);
	}

	/**
	 * The queue action for turning the robot until it faces the target given by Limelight.
	 * @param timeEnd the designated time for the command to end
	 */
	public static void queueAngle_To_Limelight_X(double timeEnd) {
		double rotValue = PIDLimelightXRot.calculate(Robot.limelight.getTargetX(), 0);	// calculate rotation input to a PID loop
		rotValue = MathUtil.clamp(rotValue, -.29, .29);						// make sure robot doesn't attempt to rotate too fast
		Robot.overrideRCW = -rotValue;
	}

	/**
	 * The queue action for pivoting the shooter to a certain encoder value.
	 * @param timeEnd the designated time for the command to end
	 * @param param1 the first parameter, the encoder value to set the shooter to
	 */
	public static void queueShooter_Pivot(double timeEnd, double param1) {
		Robot.shooter.setPivotPosition(param1);
	}
}