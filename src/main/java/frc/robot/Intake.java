// FRC TEAM 4855 ROBOT CODE
// 2020 GAME INFINITE RECHARGE

// Swerve-bot code: Intake class

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;

public class Intake {
    private Spark pivot;
    private Spark wheels;
    public Encoder encoder; // TODO make private
    
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

    public void stopIntakeWheels() {
        wheels.set(0);
    }

    public void setPivot(double speed) {
        pivot.set(speed);
    }

    public void stopPivot() {
        pivot.set(0);
    }

    public void stop() {
        wheels.set(0);
        pivot.set(0);
    }

    public double getPivotPosition() {
        return encoder.get();
    }
}