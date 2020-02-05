// FRC TEAM 4855 ROBOT CODE
// 2020 GAME INFINITE RECHARGE

// Swerve-bot code: Fan-style flywheel shooter class

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
    // Define instance variables
    private double velocitySetpoint = 0;        // the desired velocity setpoint for the flywheel
    private int speedUpTime = -1;               // the amount of program ticks remaining for the shooter to be fed by percentage input
    private boolean isRunning = false;          // whether or not the flywheel is running
    private final int MAX_SPEEDUP_TICKS = 90;   // the flywheel will have 90 ticks to increase the speed to roughly where it needs to be via percent output
    private final int ACCEPTABLE_ERROR = 40;    // the acceptable error to reasonably state that the shooter is ready to shoot
    private enum Phase {
        OFF, SPEED_UP, LOCK_IN
    }
    private Phase currentPhase = Phase.OFF;
    double kP = .00044;     // originally .00044
    double kI = 0;
    double kD = .0034;
    double kF = .000173;    // originally .000173

    // Define hardware
    private VictorSP feeder;
    private Spark pivot;
    private CANSparkMax flywheel; 
    private CANPIDController PID;
    private CANEncoder encoder;

    /**
     * Constructs the Shooter class.
     * @param sparkMaxId the ID of the CAN Spark Max that the flywheel runs off of
     * @param feederId the PWM ID of the feeder motor controller
     */
    public Shooter(int sparkMaxId, int feederId, int pivotId) {
        flywheel = new CANSparkMax(sparkMaxId, MotorType.kBrushless);
        feeder = new VictorSP(feederId);
        encoder = flywheel.getEncoder();
        pivot = new Spark(2);
        PID = flywheel.getPIDController();
        PID.setOutputRange(-1, 1);
        PID.setP(kP);
        PID.setI(kI);
        PID.setD(kD);
        PID.setFF(kF);
    }

    /**
     * Sets the flywheel to a specified velocity. Returns false until the shooter has reached the acceptable speed
     * @param setpoint the desired velocity to set the flywheel to
     * @return true or false depending on the speed has been locked in
     */
    public boolean setFlywheelSpeed(double setpoint) {
        velocitySetpoint = setpoint;
        isRunning = true;
        switch (currentPhase) {
            case SPEED_UP:
                SmartDashboard.putString("Phase", "SPEED_UP");
                break;
            case OFF:
                SmartDashboard.putString("Phase", "OFF");
                break;
            case LOCK_IN:
                SmartDashboard.putString("Phase", "LOCK_IN");
                break;
            default:
                SmartDashboard.putString("Phase", "???");
                break;
        }
        if (currentPhase == Phase.OFF) {
            currentPhase = Phase.SPEED_UP;
            speedUpTime = MAX_SPEEDUP_TICKS;
        }
        // TODO getting consistent error messages: [CAN SPARK MAX] timed out while waiting for Periodic Status 1, Periodic Status 1
        if (speedUpTime > -1 && currentPhase == Phase.SPEED_UP) {
            double percentOutput = -((setpoint / 5100) + .015); // formerly (setpoint / 5100) - .025
            flywheel.set(percentOutput);
            speedUpTime --;
            if (speedUpTime <= 0) {
                speedUpTime = -1;
                currentPhase = Phase.LOCK_IN;
            }
        }
        if (currentPhase == Phase.LOCK_IN) {
            PID.setReference(-setpoint, ControlType.kVelocity);
            if (Math.abs(setpoint + encoder.getVelocity()) < ACCEPTABLE_ERROR) return true; else return false;
        }
        return false;
    }

    /**
     * Shuts off the flywheel. This should be used every time the shooter needs to stop shooting.
     */
    public void killFlywheel() {
        currentPhase = Phase.OFF;
        speedUpTime = -1;
        flywheel.set(0);
        isRunning = false;
        velocitySetpoint = 0;
    }

    /**
     * Takes a distance from the lidar sensor and converts it to a velocity setpoint
     * @param dist the distance in inches
     * @return a double of the setpoint
     */
    public double getVelocityFromDistance(double dist) {
        return 0;
    }

    /**
     * Gets the raw velocity reading from the flywheel encoder.
     * @return a double of the encoder velocity
     */
    public double getFlywheelVelocity() {
        return encoder.getVelocity();
    }

    /**
     * Gets the velocity setpoint for the flywheel.
     * @return a double for the desired RPM of the flywheel
     */
    public double getFlywheelSetpoint() {
        return velocitySetpoint;
    }

    public double getFlywheelCurrent() {
        return flywheel.getBusVoltage();
    }

    /**
     * Return whether or not the flywheel is running.
     * @return true if the flywheel is running, false if not
     */
    public boolean isRunning() {
        return isRunning;
    }

    /**
     * Runs the feeder mechanism. This is done manually for now.
     * @param speed the speed to run the feeder motor at
     */
    public void runFeeder(double speed) {
        feeder.set(speed);
    }

    /**
     * Kills the feeder motor.
     */
    public void killFeeder() {
        feeder.set(0);
    }

    /**
     * Sets the speed of the pivot arm. Will eventually work exclusively with setpoints.
     * @param speed the percent speed to set the motor to
     */
    public void setPivot(double speed) {
        pivot.set(speed);
    }

    /**
     * Kills the pivot motor
     */
    public void killPivot() {
        pivot.set(0);
    }
}