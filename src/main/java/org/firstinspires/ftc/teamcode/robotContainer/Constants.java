package org.firstinspires.ftc.teamcode.robotContainer;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class Constants
{
    // enum to specify opMode type
    public enum OpModeType {
        TELEOP,
        BLUE_RIGHT_AUTO,
        BLUE_LEFT_AUTO,
        RED_RIGHT_AUTO,
        RED_LEFT_AUTO,
        BLUE_LEFT_2_0,
        BLUE_RIGHT_2_0,
        RED_RIGHT_2_0,
        RED_LEFT_2_0
    }

    public static double H_START_BLUE = 90;
    public static double H_END_BLUE = 105;

    public static double H_START_RED = 0;
    public static double H_END_RED = 9;

    public static int LOW = 1200;
    public static int MEDIUM = 1300;
    public static int HIGH = 2000;
    public static int LOWLOW = 800;
    public static int AUTON = 1000;

    public static double LEFT_SCORE_POSITION_X = 20;
    public static double LEFT_SCORE_POSITION_Y = 20;
    public static double LEFT_SCORE_POSITION_0 = 20;

    public static double RIGHT_SCORE_POSITION_X = -0.02;
    public static double RIGHT_SCORE_POSITION_Y = 0.04;
    public static double RIGHT_SCORE_POSITION_0 = 0.13;

    public static double MIDDLE_SCORE_POSITION_X = -0.02;
    public static double MIDDLE_SCORE_POSITION_Y = 0.04;
    public static double MIDDLE_SCORE_POSITION_0 = -1;

    public static double DEFAULT_INTAKE_SPEED = 1.0;

    public enum OperatorMode {
        SINGLE_OPERATOR_MODE,
        DOUBLE_OPERATOR_MODE
    }
}
