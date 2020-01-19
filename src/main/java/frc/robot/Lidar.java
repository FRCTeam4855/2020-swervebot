package frc.robot;

import edu.wpi.first.wpilibj.Counter;

public class Lidar {
    private Counter device;
    private double off = 0;         // the measurable offset
    public enum Unit {
        INCHES, CENTIMETERS
    }

    public Lidar() {
        device = new Counter(1);
        device.setMaxPeriod(1.00);
        device.setSemiPeriodMode(true);
        device.reset();
    }

    public double getDistance(Unit units) {
        double dist = 0;
        if (device.get() < 1) {
            dist = 0;
        } else {
            dist = (device.getPeriod()*1000000.0/10.0) - off;
        }
        if (units == Unit.INCHES) return dist * .393701; else return dist;
    }
}