package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Base64;

public class PoseEstimationSubsystem extends SubsystemBase {

    HardwareMap m_hardwareMap;
    private IMU m_imu;
    Telemetry m_telemetry;
    Motor.Encoder leftEncoder;
    Motor.Encoder rightEncoder;
    Motor.Encoder centerEncoder;

    double delta_left_encoder_pos;
    double left_encoder_pos;
    double prev_left_encoder_pos;

    double delta_right_encoder_pos;
    double right_encoder_pos;
    double prev_right_encoder_pos;

    double delta_center_encoder_pos;
    double center_encoder_pos;
    double prev_center_encoder_pos;

    public static double trackwidth;

    double phi;

    double delta_x;
    double delta_y;

    double x_pos;
    double y_pos;

    double delta_middle_pos;
    double delta_prep_pos;

    double heading;




    public PoseEstimationSubsystem(HardwareMap hardwareMap, Telemetry telemetry)
    {
        m_hardwareMap = hardwareMap;
        m_telemetry = telemetry;
        m_imu = m_hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        m_imu.initialize(new IMU.Parameters(orientationOnRobot));

        leftEncoder = m_hardwareMap.get(Motor.Encoder.class, "leftEncoder");
        rightEncoder = m_hardwareMap.get(Motor.Encoder.class, "rightEncoder");
        centerEncoder = m_hardwareMap.get(Motor.Encoder.class, "centerEncoder");
    }

//    public Pose2d positionEstimation(){
//
//    }

    public Pose2d getPose()
    {
        double heading = m_imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return new Pose2d(new Translation2d(0.0,0.0),new Rotation2d(heading));
    }

    @Override
    public void periodic()
    {
        m_telemetry.addData("RobotAngle", getPose().getHeading());
        m_telemetry.update();
    }
}
