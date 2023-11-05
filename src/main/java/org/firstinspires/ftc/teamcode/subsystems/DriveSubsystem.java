package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.UnscaledMecanumDriveKinematics;

import java.util.Arrays;
import java.util.List;

/**
 * state machine for the mecanum drive. All movement/following is async to fit the paradigm.
 */
@Config
public class DriveSubsystem extends SubsystemBase {

    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    UnscaledMecanumDriveKinematics m_normalizedMecanumDriveKinematics;
    UnscaledMecanumDriveKinematics m_mecanumDriveKinematics;
    DcMotorEx frontLeft, frontRight, rearLeft, rearRight;
    public static double kp = 10.0;
    private List<DcMotorEx> m_motorList;

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry)
    {
        m_hardwareMap=hardwareMap;
        m_telemetry=telemetry;

        //This kinematics object is used for unnormalized speed control (i.e. it takes into accounts the robot frame dimensions) for precise auton
        m_mecanumDriveKinematics = new UnscaledMecanumDriveKinematics(
                new Translation2d(Constants.WHEEL_BASE/2.0, Constants.TRACK_WIDTH/2.0),
                new Translation2d(Constants.WHEEL_BASE/2.0 , -Constants.TRACK_WIDTH/2.0),
                new Translation2d(-Constants.WHEEL_BASE/2.0, Constants.TRACK_WIDTH/2.0),
                new Translation2d(-Constants.WHEEL_BASE/2.0, -Constants.TRACK_WIDTH/2.0));

        //This kinematics object is used for normalized speed control (doesn't care about robot dimensions) for teleop
        m_normalizedMecanumDriveKinematics = new UnscaledMecanumDriveKinematics(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5)); // NOTE: these should not change

        frontLeft = m_hardwareMap.get(DcMotorEx.class, "FL");
        frontRight = m_hardwareMap.get(DcMotorEx.class, "FR");
        rearLeft = m_hardwareMap.get(DcMotorEx.class, "BL");
        rearRight = m_hardwareMap.get(DcMotorEx.class, "BR");

        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        m_motorList = Arrays.asList(frontLeft, frontRight, rearLeft, rearRight);

        setInitialMotorConfiguration();
        setMotorZeroBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); //Auton Defaults
    }

    private void setInitialMotorConfiguration()
    {

        for (DcMotorEx motor : m_motorList) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(Constants.MAX_ACHIEVABLE_RPM_FRACTION);
            motorConfigurationType.setMaxRPM(Constants.MAX_RPM);
            motorConfigurationType.setTicksPerRev(Constants.TICKS_PER_REV / Constants.GEAR_RATIO);
            motor.setMotorType(motorConfigurationType);
        }
    }

    public void setPController(double p)
    {
        for (DcMotorEx motor : m_motorList)
        {
            motor.setPositionPIDFCoefficients(kp);
        }
    }

    public void setMotorZeroBehavior(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior)
    {
        for (DcMotorEx motor : m_motorList)
        {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setMotorMode(DcMotorEx.RunMode runMode)
    {
        for (DcMotorEx motor : m_motorList)
        {
            motor.setMode(runMode);
        }
    }

    public void drive(double leftX, double leftY, double rightX, double throttle, double currentHeading)
    {
        double throttleSlope = 1 - Constants.THROTTLEMINLEVEL;
        double throttleScale = throttleSlope * throttle + Constants.THROTTLEMINLEVEL;

        MecanumDriveWheelSpeeds wheelSpeeds = m_normalizedMecanumDriveKinematics.toWheelSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        leftY * throttleScale,
                        leftX * throttleScale,
                        rightX * throttleScale,
                        new Rotation2d(currentHeading)));
        wheelSpeeds.normalize(1.0);
        setMotorPowers(wheelSpeeds);
    }
    public void drive(double leftX, double leftY, double rightX, double currentHeading)
    {


        MecanumDriveWheelSpeeds wheelSpeeds = m_normalizedMecanumDriveKinematics.toWheelSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        leftY,
                        leftX ,
                        rightX,
                        new Rotation2d(currentHeading)));
        wheelSpeeds.normalize(1.0);
        setMotorPowers(wheelSpeeds);
    }


    private void setMotorPowers(MecanumDriveWheelSpeeds wheelSpeeds)
    {
        frontLeft.setPower(wheelSpeeds.frontLeftMetersPerSecond);
        frontRight.setPower(wheelSpeeds.frontRightMetersPerSecond);
        rearLeft.setPower(wheelSpeeds.rearLeftMetersPerSecond);
        rearRight.setPower(wheelSpeeds.rearRightMetersPerSecond);
    }

    public void setMotorPowers(double power)
    {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        rearLeft.setPower(power);
        rearRight.setPower(power);
    }

    public void setMovement(Translation2d inputDisp, double angleDispDegrees)
    {
        //We use the kinematics model to convert desired chassis displacements (in m)
        //over to desired wheel displacements (also in m)
        MecanumDriveWheelSpeeds wheelDisp =
                m_normalizedMecanumDriveKinematics.toWheelSpeeds(
                        new ChassisSpeeds(inputDisp.getX(),
                                inputDisp.getY(),
                                angleDispDegrees * Math.PI/180.0));

        //Convert the linear displacements to angular displacements (in units of rotations)
        double frontLeftAngDisp = wheelDisp.frontLeftMetersPerSecond / Constants.WHEEL_RADIUS / (2.0 * Math.PI);
        double frontRightAngDisp = wheelDisp.frontRightMetersPerSecond / Constants.WHEEL_RADIUS / (2.0 * Math.PI);
        double rearLeftAngDisp = wheelDisp.rearLeftMetersPerSecond / Constants.WHEEL_RADIUS / (2.0 * Math.PI);
        double rearRightAngDisp = wheelDisp.rearRightMetersPerSecond / Constants.WHEEL_RADIUS / (2.0 * Math.PI);

        //Convert wheel rotations to wheel encoder Ticks
        int frontLeftTicks = (int)(frontLeftAngDisp * Constants.TICKS_PER_REV);
        int frontRightTicks = (int)(frontRightAngDisp * Constants.TICKS_PER_REV);
        int rearLeftTicks = (int)(rearLeftAngDisp * Constants.TICKS_PER_REV);
        int rearRightTicks = (int)(rearRightAngDisp * Constants.TICKS_PER_REV);

        //Apply the computed encoder tick rotations to the wheel position PID controller
        setMotorMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setTargetPosition(frontLeftTicks);
        frontRight.setTargetPosition(frontRightTicks);
        rearLeft.setTargetPosition(rearLeftTicks);
        rearRight.setTargetPosition(rearRightTicks);
        setMotorMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public boolean isBusy()
    {
        return (frontLeft.isBusy() && frontRight.isBusy() && rearLeft.isBusy() && rearRight.isBusy());
    }

//    private Vector2d quadraticControlLaw(Vector2d inputVec) //TODO: use this only if the controls are too sensitive
//    {
//        double outX = inputVec.getX() * inputVec.getX();
//        double outY = inputVec.getY() * inputVec.getY();
//        return new Vector2d(outX, outY);
//    }

    @Override
    public void periodic()
    {
        setPController(kp);
//        m_telemetry.addData("P controller", kp);
//        m_telemetry.addData("LF Encoder", frontLeft.getCurrentPosition());
//        m_telemetry.addData("LR Encoder", frontRight.getCurrentPosition());
//        m_telemetry.addData("RF Encoder", rearLeft.getCurrentPosition());
//        m_telemetry.addData("RR Encoder", rearRight.getCurrentPosition());
//
//        m_telemetry.update();
    }

}