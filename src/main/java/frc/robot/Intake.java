// FRC TEAM 4855 ROBOT CODE
// 2020 GAME INFINITE RECHARGE

// Swerve-bot code: Intake class

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;

public class Intake {
    private Spark pivot;
    private Spark wheels;
    private Encoder encoder;
    
    /**
     * Constructs an instance of Intake. It consists of a pivot arm and intake wheels, both driven by Spark motor controllers.
     * @param pivotPort the PWM port for the pivot arm
     * @param wheelsPort the PWM port for the intake wheels
     */
    public Intake(int pivotPort, int wheelsPort, int encPort1, int encPort2, int encPort3) {
        pivot = new Spark(pivotPort);
        wheels = new Spark(wheelsPort);
        encoder = new Encoder(encPort1, encPort2, encPort3);
    }

    /**
     * Sets the intake wheels to a specified speed.
     * @param speed percent speed
     */
    public void setIntakeWheels(double speed) {
        wheels.set(speed);
    }

    /**
     * Stops the intake wheels.
     */
    public void stopIntakeWheels() {
        wheels.set(0);
    }

    /**
     * Sets the pivot arm to a certain speed.
     * @param speed the speed to set the pivot arm to
     */
    public void setPivot(double speed) {
        pivot.set(speed);
    }

    /**
     * Stops the pivot motor.
     */
    public void stopPivot() {
        pivot.set(0);
    }

    /**
     * Stops both the pivot and the intake wheel motors.
     */
    public void stop() {
        wheels.set(0);
        pivot.set(0);
    }

    /**
     * Gets the encoder position of the pivot motor
     * @return encoder position
     */
    public double getPivotPosition() {
        return encoder.get();
    }
}