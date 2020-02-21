// FRC TEAM 4855 ROBOT CODE
// 2020 GAME INFINITE RECHARGE

// Swerve-bot code: Action Queue class

package frc.robot;

import java.util.concurrent.TimeUnit;

/**
 * A single list of commands that are executed by the robot whenever necessary.
 */
public class ActionQueue {

	enum Command {
		DEAD, WAIT_FOR_SENSOR, PREPARE_TURN, SWERVE, DRIVE_STRAIGHT, TURN_TO_ANGLE, FEED_BALL, SHOOTER_PIVOT, INTAKE_PIVOT, RUN_FLYWHEEL, RUN_INTAKE_WHEELS, ANGLE_TO_LIMELIGHT_X;
	}

	Command queueListActions [] = new Command [20];		// action ID to perform
	double queueListTimeStart [] = new double [20];		// elapsed begin time to run a command
	double queueListTimeEnd [] = new double [20];		// elapsed begin time to end a command
	boolean queueListKillMotor[] = new boolean [20];	// whether to kill designated motors after the command is stopped or not
	double queueListParam1 [] = new double [20];		// parameter 1 for queue item
	double queueListParam2 [] = new double [20];		// parameter 2 for queue item
	double queueListParam3 [] = new double [20];		// parameter 3 for queue item
	
	private double queueElapsedTime = 0;				// current elapsed time for this command in seconds
	private double queueStartTime = 0;					// the system time during which this command started in nanoseconds
	private double queueTotalHangTime = 0;				// when checking for sensors, the queue pauses, and this command holds how much time the queue has been paused for
	private boolean queueIsRunning = false;				// if queue is enabled or not
	private boolean queueUsesSwerveOverrides = false;	// whether or not the action queue makes use of the swerve() overrides

	private double queueMaxTime = -1;					// largest end time in queue
	private int queueLength = 0;						// queue length
	
	/**
	 * Constructs an ActionQueue.
	 * @param overrides whether or not the action queue should zero out any swerve overrides when it is complete, it's a
	 * good idea to set this to true if you are using the swerves during your action queue
	 */
	public ActionQueue(boolean overrides) {
		queueUsesSwerveOverrides = overrides;
	}

	/**
	 * Feeds the queue a new command. Commands can be assigned in any order.
	 * @param action the action ID to feed
	 * @param timeStart the elapsed time within the queue in which to start the command
	 * @param timeEnd the elapsed time within the queue in which to end the command
	 * @param killMotor whether or not an action's associated motors should be set to 0 after the action has completed
	 * @param param1 the 1st parameter
	 * @param param2 the 2nd parameter
	 * @param param3 the 3rd parameter
	 */
	public void queueFeed(Command action, double timeStart, double timeEnd, boolean killMotor, double param1, double param2, double param3) {
		queueListActions[queueLength] = action;
		queueListTimeStart[queueLength] = timeStart;
		queueListTimeEnd[queueLength] = timeEnd;
		queueListKillMotor[queueLength] = killMotor;
		queueListParam1[queueLength] = param1;
		queueListParam2[queueLength] = param2;
		queueListParam3[queueLength] = param3;
		queueLength ++;
	}
	
	/**
	 * In the unlikely event that a queue entry must be deleted, this function will delete an entry at a certain timeframe.
	 * @param action the action ID to kill
	 * @param timeStart the starting time of the command to kill
	 */
	public void queueDelete(Command action, int timeStart) {
		for (int i = 0; i < queueLength; i ++) {
			if (queueListActions[i] == action && queueListTimeStart[i] == timeStart) {
				// The array will still contain a corpse even though the command is deleted
				queueListActions[i] = Command.DEAD;
				queueListTimeStart[i] = -1;
				queueListTimeEnd[i] = -1;
				break;
			}
		}
	}
	
	/**
	 * Starts the queue. If this command is run and the queue is already running, the queue will instead stop.
	 */
	public void queueStart() {
		if (!queueIsRunning) {
			queueIsRunning = true;
			queueStartTime = System.nanoTime();
			queueElapsedTime = 0;
			queueTotalHangTime = 0;
		} else queueStop();
	}
	
	/**
	 * Stops the queue.
	 */
	public void queueStop() {
		queueIsRunning = false;
		queueElapsedTime = 0;
		if (queueUsesSwerveOverrides) {
			Robot.overrideFWD = 0;
			Robot.overrideSTR = 0;
			Robot.overrideRCW = 0;
		}
	}
	
	/**
	 * Returns if the queue is running or not.
	 * @return true or false based on if the queue is running
	 */
	public boolean queueRunning() {
		return queueIsRunning;
	}

	/**
	 * This function runs through the fed commands, increases elapsed time, and runs robot commands.
	 */
	public void queueRun() {
		boolean allowTimePassage = true;	// whether or not to allow the passage of time to proceed
		for (int i = 0; i < queueLength; i ++) {
			if (queueListTimeEnd[i] > queueMaxTime) queueMaxTime = queueListTimeEnd[i];
            if (queueListTimeStart[i] <= queueElapsedTime && queueElapsedTime <= queueListTimeEnd[i]) {
                // Run a certain action. Parameters will be shipped to the handler class along with the command.
                switch (queueListActions[i]) {
					case WAIT_FOR_SENSOR:
						if (!ActionQueueHandler.queueCheck_Sensor(queueListTimeEnd[i], queueListParam1[i], queueListParam2[i], queueListParam3[i])) {
							// Sensors are not ready, hang the queue
							queueTotalHangTime += TimeUnit.SECONDS.convert(System.nanoTime(), TimeUnit.NANOSECONDS) - queueListTimeStart[i];
							allowTimePassage = false;
						} else allowTimePassage = true;
						break;
                    case PREPARE_TURN:
                        ActionQueueHandler.queuePrepare_Turn(queueListTimeEnd[i],queueListParam1[i],queueListParam2[i]);
                        break;
                    case SWERVE:
                        ActionQueueHandler.queueSwerve(queueListTimeEnd[i],queueListParam1[i],queueListParam2[i],queueListParam3[i]);
						break;
					case DRIVE_STRAIGHT:
						ActionQueueHandler.queueDrive_Straight(queueListTimeEnd[i],queueListParam1[i], queueListParam2[i]);
						break;
					case TURN_TO_ANGLE:
						ActionQueueHandler.queueTurn_To_Angle(queueListTimeEnd[i], queueListParam1[i]);
						break;
					case FEED_BALL:
						ActionQueueHandler.queueFeed_Ball(queueListTimeEnd[i]);
						break;
					case RUN_FLYWHEEL:
						ActionQueueHandler.queueRun_Flywheel(queueListTimeEnd[i], queueListParam1[i], queueListParam2[i]);
						break;
					case ANGLE_TO_LIMELIGHT_X:
						ActionQueueHandler.queueAngle_To_Limelight_X(queueListTimeEnd[i]);
						break;
					case RUN_INTAKE_WHEELS:
						ActionQueueHandler.queueRun_Intake_Wheels(queueListTimeEnd[i], queueListParam1[i]);
						break;
					case SHOOTER_PIVOT:
						ActionQueueHandler.queueShooter_Pivot(queueListTimeEnd[i], queueListParam1[i]);
						break;
					case INTAKE_PIVOT:
						ActionQueueHandler.queueIntake_Pivot(queueListTimeEnd[i], queueListParam1[i]);
					default:
                        break;
                }
			}
			if (queueElapsedTime == queueListTimeEnd[i] + 1 && queueListKillMotor[i]) {
				// Kill the corresponding motor if applicable
				switch (queueListActions[i]) {
					case FEED_BALL:
						Robot.shooter.killFeeder();
						break;
					case RUN_FLYWHEEL:
						Robot.shooter.killFlywheel();
						break;
					case RUN_INTAKE_WHEELS:
						Robot.intake.stopIntakeWheels();
						break;
					case INTAKE_PIVOT:
						Robot.intake.stopPivot();
						break;
					case DRIVE_STRAIGHT:
						ActionQueueHandler.resetNeos = false;
						break;
					default:
						break;
				}
			}
		}
		// Pass time
		if (allowTimePassage) queueElapsedTime = TimeUnit.SECONDS.convert((long) (System.nanoTime() - queueStartTime), TimeUnit.NANOSECONDS) - queueTotalHangTime;	// convert system time to seconds
		if (queueMaxTime < queueElapsedTime) queueStop();	// if the last command has finished, the queue can stop
	}
}