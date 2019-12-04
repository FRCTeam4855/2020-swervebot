// FRC TEAM 4855 ROBOT CODE
// 2020 GAME INFINITE RECHARGE

// Swerve-bot code: Action Queue Handler class

package frc.robot;

/**
 * Runs all ActionQueue instances and is responsible for holding methods that run the robot and holding onto all ActionQueue objects.
 */
public class ActionQueueHandler {
    private ActionQueue[] queues;
    
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
	 * The queue action for preparing a turn. This is functionally similar to the queueSwerve command
	 * @param timeEnd the designated time for the command to end
	 * @param param1 the first parameter, the fwd input
	 * @param param2 the second parameter, the str input
	 */
	public static void queuePrepare_Turn(int timeEnd, double param1, double param2) {
		Robot.swerve(param1,param2,0,false);
	}

	/**
	 * The queue action for swerving in its raw form. This is completed relative to the ROBOT.
	 * @param timeEnd the designated time for the command to end
	 * @param param1 the first parameter, FWD
	 * @param param2 the second parameter, STR
	 * @param param3 the third parameter, RCW
	 */
	public static void queueSwerve(int timeEnd, double param1, double param2, double param3) {
		Robot.swerve(param1,param2,param3,false);
	}
}