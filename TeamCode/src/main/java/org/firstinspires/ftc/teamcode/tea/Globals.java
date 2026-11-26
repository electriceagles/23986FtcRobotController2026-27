package org.firstinspires.ftc.teamcode.tea;

import com.bylazar.configurables.annotations.Configurable;

/**
 * The Globals file is only for you guys to store all your variables.
 * Instead of making new tuning variables for anything you can store them here and just import from here.
 * Also makes tuning easier since if you change a variable here it'll change for all other files.
 *
 * Hope this makes your progress smoother and simpler.
 */
@Configurable
public class Globals {

    // Make sure to keep them public static so they can be imported easily
    // with static you don't have to initialize this file in any other file either.
    public static double P = 120.0400;
    public static double F = 14.9600; // These two variables are statically imported in RobotHardware just to let you know.

    public static double kP = 1.6;
    public static double kD = 0.12;
    public static double kF = 0.0;
    public static double turMaxPow = 0.8;
    public static double deadbandDeg = 0.5;

    // Feel free to add more variables here for anything else, I just added these as a starter
}

