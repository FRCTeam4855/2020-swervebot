// FRC TEAM 4855 ROBOT CODE
// 2020 GAME INFINITE RECHARGE

// Swerve-bot code: Lidar range finder class

package frc.robot;

import edu.wpi.first.wpilibj.Counter;

public class Lidar {
    private Counter device;
    private double off = 0;         // the measurable offset
    public enum Unit {
        INCHES, CENTIMETERS
    }

    public Lidar(int port) {
        device = new Counter(port);
        device.setMaxPeriod(1.00);
        device.setSemiPeriodMode(true);
        device.reset();
    }

    /**
     * This command gets the distance to the nearest object.
     * @param units an enum of either inches or centimeters
     * @return double in the desired unit of distance
     */
    public double getDistance(Unit units) {
        double dist = 0;
        if (device.get() < 1) {
            dist = 0;
        } else {
            dist = (device.getPeriod() * 1000000.0 / 10.0) - off;
        }
        if (units == Unit.INCHES) return dist * .393701; else return dist;
    }
}