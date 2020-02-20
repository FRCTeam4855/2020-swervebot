// FRC TEAM 4855 ROBOT CODE
// 2020 GAME INFINITE RECHARGE

// Swerve-bot code: Blinkin LED Driver class

package frc.robot;

import edu.wpi.first.wpilibj.Spark;

public class Blinkin {
    // Assign light patterns to class constants
    final static double RAINBOW_RAINBOWPALETTE = -.99;
    final static double RAINBOW_PARTYPALETTE = -.97;
    final static double RAINBOW_OCEANPALETTE = -.95;
    final static double RAINBOW_LAVAPALETTE = -.93;
    final static double RAINBOW_FORESTPALETTE = -.91;
    final static double RAINBOW_GLITTER = -.89;
    final static double CONFETTI = -.87;
    final static double SHOT_RED = -.85;
    final static double SHOT_BLUE = -.83;
    final static double SHOT_WHITE = -.81;
    final static double SINELON_RAINBOWPALETTE = -.79;
    final static double SINELON_PARTYPALETTE = -.77;
    final static double SINELON_OCEANPALETTE = -.75;
    final static double SINELON_LAVAPALETTE = -.73;
    final static double SINELON_FORESTPALETTE = -.71;
    final static double BPM_RAINBOWPALETTE = -.69;
    final static double BPM_PARTYPALETTE = -.67;
    final static double BPM_OCEANPALETTE = -.65;
    final static double BPM_LAVAPALETTE = -.63;
    final static double BPM_FORESTPALETTE = -.61;
    final static double FIRE_MEDIUM = -.59;
    final static double FIRE_LARGE = -.57;
    final static double TWINKLES_RAINBOWPALETTE = -.55;
    final static double TWINKLES_PARTYPALETTE = -.53;
    final static double TWINKLES_OCEANPALETTE = -.55;
    final static double TWINKLES_LAVAPALETTE = -.53;
    final static double TWINKLES_FORESTPALETTE = -.51;
    final static double COLORWAVES_RAINBOWPALETTE = -.45;
    final static double COLORWAVES_PARTYPALETTE = -.43;
    final static double COLORWAVES_OCEANPALETTE = -.41;
    final static double COLORWAVES_LAVAPALETTE = -.39;
    final static double COLORWAVES_FORESTPALETTE = -.37;
    final static double LARSONSCAN_RED = -.35;
    final static double LARSONSCAN_GRAY = -.33;
    final static double LIGHTCHASE_RED = -.31;
    final static double LIGHTCHASE_BLUE = -.29;
    final static double LIGHTCHASE_GRAY = -.27;
    final static double HEARTBEAT_RED = -.25;
    final static double HEARTBEAT_BLUE = -.23;
    final static double HEARTBEAT_WHITE = -.21;
    final static double HEARTBEAT_GRAY = -.19;
    final static double BREATH_RED = -.17;
    final static double BREATH_BLUE = -.15;
    final static double BREATH_GRAY = -.13;
    final static double STROBE_RED = -.11;
    final static double STROBE_BLUE = -.09;
    final static double STROBE_GOLD = -.07;
    final static double STROBE_WHITE = -.05;
    final static double C1_END_TO_END_BLEND_TO_BLACK = -.03;
    final static double C1_LARSONSCAN = -.01;
    final static double C1_LIGHTCHASE = .01;
    final static double C1_HEARTBEAT_SLOW = .03;
    final static double C1_HEARTBEAT_MEDIUM = .05;
    final static double C1_HEARTBEAT_FAST = .07;
    final static double C1_BREATH_SLOW = .09;
    final static double C1_BREATH_FAST = .11;
    final static double C1_SHOT = .13;
    final static double C1_STROBE = .15;
    final static double C2_END_TO_END_BLEND_TO_BLACK = .17;
    final static double C2_LARSONSCAN = .19;
    final static double C2_LIGHTCHASE = .21;
    final static double C2_HEARTBEAT_SLOW = .23;
    final static double C2_HEARTBEAT_MEDIUM = .25;
    final static double C2_HEARTBEAT_FAST = .27;
    final static double C2_BREATH_SLOW = .29;
    final static double C2_BREATH_FAST = .31;
    final static double C2_SHOT = .33;
    final static double C2_STROBE = .35;
    final static double SPARKLE_C1_ON_C2 = .37;
    final static double SPARKLE_C2_ON_C1 = .39;
    final static double C1_AND_C2_GRADIENT = .41;
    final static double C1_AND_C2_BPM = .43;
    final static double C1_AND_C2_END_TO_END_BLEND = .45;
    final static double END_TO_END_BLEND = .47; // ???
    final static double C1_AND_C2_NO_BLEND = .49;
    final static double C1_AND_C2_TWINKLES = .51;
    final static double C1_AND_C2_COLOR_WAVES = .53;
    final static double C1_AND_C2_SINELON = .55;
    final static double HOT_PINK = .57;
    final static double DARK_RED = .59;
    final static double RED = .61;
    final static double RED_ORANGE = .63;
    final static double ORANGE = .65;
    final static double GOLD = .67;
    final static double YELLOW = .69;
    final static double LAWN_GREEN = .71;
    final static double LIME = .73;
    final static double DARK_GREEN = .75;
    final static double GREEN = .77;
    final static double BLUE_GREEN = .79;
    final static double AQUA = .81;
    final static double SKY_BLUE = .83;
    final static double DARK_BLUE = .85;
    final static double BLUE = .87;
    final static double BLUE_VIOLET = .89;
    final static double VIOLET = .91;
    final static double WHITE = .93;
    final static double GRAY = .95;
    final static double DARK_GRAY = .97;
    final static double BLACK = .99;

    double pattern = C1_AND_C2_SINELON;
    Spark leds;

    /**
     * Creates an instance of the Blinkin driver.
     * @param s the PWM port of the driver.
     */
    public Blinkin(int s) {
        leds = new Spark(s);
    }

    /**
     * Sets the LEDs to a certain color. This function won't work when the robot is disabled.
     * @param color the color to set the lights to, coordinated with the constants set as static in the Blinkin class
     */
    public void setLEDs(double color) {
        pattern = color;
        leds.set(color);
    }

    /**
     * Gets the current display pattern of the LEDs.
     * @return a number between -1 and 1 corresponding to the current pattern
     */
    public double getLEDs() {
        return pattern;
    }
}