package org.firstinspires.ftc.teamcode;

public final class Constants
{
    // enum to specify opMode type
    public enum OpModeType {
        TELEOP,
        BLUE_RIGHT_AUTO,
        BLUE_LEFT_AUTO,
        RED_RIGHT_AUTO,
        RED_LEFT_AUTO
    }

    public static boolean FIELD_CENTRIC_ENABLED = true;
    public static boolean ROBOT_CENTRIC_ENABLED = false;


    //LINEAR SLIDE LEVELS
    public static int LOW = 0;
    public static int MEDIUM = 300;
    public static int HIGH = 2700;

    public static double DEFAULT_INTAKE_SPEED = 0.9;

    //For gamepad trigger throttling (in our drivetrainSS and driveCommand)
    public static double THROTTLEMINLEVEL = 0.2;

    //Drivetrain motor configuration constants (many of these primarily affect the closed loop motor behavior)
    public static final double TICKS_PER_REV = 537.7;
    public static final double MAX_RPM = 312;
    public static final double MAX_ACHIEVABLE_RPM_FRACTION = 0.6; //Should be between 0 and 1
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed

    //Constants used for mecanum kinematic modeling
    public static double WHEEL_RADIUS = 48e-3; //radius of one mecanum wheel (in m)
    public static double TRACK_WIDTH = 317.2e-3; //distance between both front wheels (in m) (or both rear wheel)
    public static double WHEEL_BASE = 240e-3; //distance between the front and rear wheel on one side (in m)


}
