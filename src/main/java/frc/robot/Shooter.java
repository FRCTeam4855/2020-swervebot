// FRC TEAM 4855 ROBOT CODE
// 2020 GAME INFINITE RECHARGE

// Swerve-bot code: Fan-style flywheel shooter class

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.wpilibj.I2C;
//import edu.wpi.first.wpilibj.util.Color; 
//import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;

public class Shooter {
    // Define instance variables
    private double velocitySetpoint = 0;        // the desired velocity setpoint for the flywheel
    private int speedUpTime = -1;               // the amount of program ticks remaining for the shooter to be fed by percentage input
    private boolean isRunning = false;          // whether or not the flywheel is running
    private final int MAX_SPEEDUP_TICKS = 50;   // the flywheel will have 50 ticks to increase the speed to roughly where it needs to be via percent output
    private final int ACCEPTABLE_ERROR = 70;    // the acceptable error to reasonably state that the shooter is ready to shoot
    private enum Phase {
        OFF, SPEED_UP, LOCK_IN
    }
    private Phase currentPhase = Phase.OFF;
    double kP = .00081;     // originally .00044
    double kI = 0;
    double kD = .0034;
    double kF = .000202;    // originally .000173

    //private boolean powerCellInShooter = false;         // whether or not a power cell is currently occupying the shooter space, only updated with color sensing method
    //private double proximityValue = 0;                  // the proximity value of the proximity sensor
    //private double ticksBetweenBalls = -1;              // how often balls are allowed to pass through balls
    //private int powerCellsLeft;                         // the number of balls remaining in the reserve

    // Define hardware
    private Spark feeder;
    private TalonSRX pivot;
    private CANSparkMax flywheel; 
    private CANPIDController PID;
    private PIDController pivotPID;
    private CANEncoder encoder;
    //public ColorSensorV3 colourSensor; 

    /**
     * Constructs the Shooter class.
     * @param sparkMaxId the ID of the CAN Spark Max that the flywheel runs off of
     * @param feederId the PWM ID of the feeder motor controller
     * @param pivotId the CAN ID of the Talon SRX that runs the pivot
     */
    public Shooter(int sparkMaxId, int feederId, int pivotId) {
        flywheel = new CANSparkMax(sparkMaxId, MotorType.kBrushless);
        feeder = new Spark(feederId);
        encoder = flywheel.getEncoder();
        pivot = new TalonSRX(pivotId);
        PID = flywheel.getPIDController();
        PID.setOutputRange(-1, 1);
        PID.setP(kP);
        PID.setI(kI);
        PID.setD(kD);
        PID.setFF(kF);
        pivot.configPeakOutputForward(.3);
        pivot.configPeakOutputReverse(-.3);
        pivotPID = new PIDController(.007, 0, 0);
        //colourSensor = new ColorSensorV3(I2C.Port.kOnboard);
        //powerCellsLeft = 3;
    }

    /**
     * Sets the flywheel to a specified velocity. Returns false until the shooter has reached the acceptable speed
     * @param setpoint the desired velocity to set the flywheel to
     * @return true or false depending on the speed has been locked in
     */
    public boolean setFlywheelSpeed(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, 0, 4500);
        velocitySetpoint = setpoint;
        isRunning = true;
        if (Robot.showDiagnostics) switch (currentPhase) {
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
            double percentOutput = MathUtil.clamp(-((setpoint / 5100) + .015), 0, 1); // formerly (setpoint / 5100) - .025
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
     * Sets the percentage output of the flywheel. Should be used conservatively.
     * @param speed the percent output of the speed
     */
    public void setFlywheelPercentSpeed(double speed) {
        if (!isRunning) flywheel.set(speed);
    }

    /**
     * Takes a distance from the lidar sensor and converts it to a velocity setpoint
     * @param dist the distance in inches
     * @return a double of the setpoint
     */
    public double getVelocityFromDistance(double dist) {
        if (dist < 90 || dist > 450) return getFlywheelVelocity(); // probably not valid, just use previous position
        return -0.008 * Math.pow(dist, 2) + 7.223 * dist + 1686;
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

    /**
     * Gets the voltage draw of the flywheel.
     * @return the voltage of the flywheel motor controller
     */
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
     * Sets the speed of the pivot arm.
     * @param speed the percent speed to set the motor to
     */
    public void setPivot(double speed) {
        pivot.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Sets the position of the pivot arm.
     * @param position the position in encoder units to set the arm to
     */
    public void setPivotPosition(double position) {
        pivot.set(ControlMode.PercentOutput, -pivotPID.calculate(getPivotPosition(), position));
    }

    /**
     * Takes a distance from the lidar sensor and converts it to a pivot position setpoint
     * @param dist the distance in inches
     * @return a double of the setpoint
     */
    public double getPivotPositionFromDistance(double dist) {
        if (dist < 90 || dist > 450) return getPivotPosition(); // probably not valid, just use previous position
        double adjust = 40;
        if (dist < 200) adjust = -75;
        //return -.006 * Math.pow(dist, 2) + 4.247 * dist + 294;    old
        //return -.007 * Math.pow(dist, 2) + 4.3 * dist + 245;
        return -.006 * Math.pow(dist, 2) + 4 * dist + 220 + adjust;
    }

    /**
     * Zeros the pivot encoder.
     */
    public void resetPivotPosition() {
        pivot.setSelectedSensorPosition(0);
    }

    /**
     * Returns the encoder position of the shooter pivot.
     * @return the position in encoder units of the shooter
     */
    public double getPivotPosition() {
        return pivot.getSelectedSensorPosition();
    }

    /**
     * Kills the pivot motor.
     */
    public void killPivot() {
        pivot.set(ControlMode.PercentOutput, 0);
    }

    public void sensePowerCells() {
        //Color detectedColour = colourSensor.getColor();

        //boolean previousCellOccupancy = powerCellInShooter;

        /*if (detectedColour.blue >= 0.11 && detectedColour.green >= .45 && detectedColour.green <= 0.55 && detectedColour.red >= 0.28 && detectedColour.red <= .34) {
            powerCellInShooter = true;
        }
        if (previousCellOccupancy && !powerCellInShooter){
            powerCellsLeft --;
        }*/

        /*proximityValue = colourSensor.getProximity();
        if (proximityValue > 500 && ticksBetweenBalls == -1) {
            powerCellInShooter = true;
        }
        if (proximityValue < 500 && ticksBetweenBalls == -1 && powerCellInShooter) {
            powerCellsLeft --;
            powerCellInShooter = false;
            ticksBetweenBalls = 24;
        }
        if (ticksBetweenBalls < -1) ticksBetweenBalls --;
    }
    
    public void setPowerCellsLeft(int powerCellsGained) {
        powerCellsLeft = powerCellsGained;
    }

    public int getPowerCellsLeft() {
        return powerCellsLeft; */
    }
}
