package frc.robot;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.VictorSP;

public class Intake {
    private Spark pivot;
    private VictorSP wheels;
    
    /**
     * Constructs an instance of Intake. The intake wheels are driven by a VictorSP and the pivot motors are driven by 2 Sparks joined by a PWM.
     * @param pivotPort
     * @param wheelsPort
     */
    public Intake(int pivotPort, int wheelsPort) {
        pivot = new Spark(pivotPort);
        wheels = new VictorSP(wheelsPort);
    }

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
}