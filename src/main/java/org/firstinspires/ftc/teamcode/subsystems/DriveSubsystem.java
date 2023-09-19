package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.math.BigDecimal;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * state machine for the mecanum drive. All movement/following is async to fit the paradigm.
 */
public class DriveSubsystem extends SubsystemBase {

    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    final double trackWidth = 8;
    final double wheelBase = 8;
    double L = trackWidth + wheelBase;
    DcMotorEx FL;
    DcMotorEx BL;
    DcMotorEx BR;
    DcMotorEx FR;
    double R = 0.66; //1/radius
    double throttleMuiltipliar;
    BNO055IMU imu;
    double robotAngle;
    double controllerAngle;
    double refinedX;
    double refinedY;

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry)
    {
        m_hardwareMap=hardwareMap;
        m_telemetry=telemetry;
         FL = hardwareMap.get(DcMotorEx.class, "FL");
         BL = hardwareMap.get(DcMotorEx.class, "BL");
       BR = hardwareMap.get(DcMotorEx.class, "BR");
        FR = hardwareMap.get(DcMotorEx.class, "FR");

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxisReference.INTRISIC)





        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        //TODO create all the motor objects here
    }

    public void drive(double leftX, double leftY, double rightX, double throttle, boolean isFieldCentric)
    {


        if (isFieldCentric){

            controllerAngle = Math.atan2(leftX, leftY);
            m_telemetry.addData("Controller angle", controllerAngle);
            refinedX = Math.cos(sub(controllerAngle, imu.getPosition()));
            refinedY = Math.sin(sub(controllerAngle, imu.getAngularOrientation()));
            FL.setPower(throttleMuiltipliar * (R * (refinedY - refinedX + rightX * (-(L)))));
            BL.setPower(throttleMuiltipliar * (R * (refinedY + refinedX + rightX * (-L))));
            BR.setPower(throttleMuiltipliar * (R * (refinedY - refinedX + rightX * (L))));
            FR.setPower(throttleMuiltipliar * (R * (refinedY + refinedX + rightX * (L))));





















        } else {
            throttleMuiltipliar = (0.7 * throttle) + 0.45;
            m_telemetry.addData("FL power", throttleMuiltipliar * (R * (leftY - leftX + rightX * (-(L)))));

            FL.setPower(throttleMuiltipliar * (R * (leftY - leftX + rightX * (-(L)))));
            BL.setPower(throttleMuiltipliar * (R * (leftY + leftX + rightX * (-L))));
            BR.setPower(throttleMuiltipliar * (R * (leftY - leftX + rightX * (L))));
            FR.setPower(throttleMuiltipliar * (R * (leftY + leftX + rightX * (L))));
            m_telemetry.addData("X", leftX);
            m_telemetry.addData("Y", leftY);
            m_telemetry.addData("Z", rightX);
            m_telemetry.addData("throttle", throttleMuiltipliar);
            //TODO update the drive function to run the motors appropriately
//        Pose2d poseEstimate = getPoseEstimate(); //TODO get the pose estimate from the poseEstimationSubsystem
//        Vector2d input_vec = new Vector2d(leftY, leftX).rotated(
//                isFieldCentric ? -(poseEstimate.getHeading() - Math.toRadians(m_allianceHeadingOffset)) :0
//        );
//
//        double throttleSlope = 1 - THROTTLEMINLEVEL;
//        double throttleScale = throttleSlope * throttle + THROTTLEMINLEVEL;
//        m_drive.setWeightedDrivePower(
//                new Pose2d(
//                        input_vec.getX() * throttleScale,
//                        input_vec.getY() * throttleScale,
//                        rightX * throttleScale
//                )
//      }  );
        }
    }

    private Vector2d quadraticControlLaw(Vector2d inputVec)
    {
        double outX = inputVec.getX() * inputVec.getX();
        double outY = inputVec.getY() * inputVec.getY();
        return new Vector2d(outX, outY);
    }


    public static double sub(double minuend, double subtrahend) {
        BigDecimal b1 = new BigDecimal(Double.toString(minuend));
        BigDecimal b2 = new BigDecimal(Double.toString(subtrahend));
        return b1.subtract(b2).doubleValue();

    }

    @Override
    public void periodic()
    {
//       m_telemetry.addData("robotPosePoseEstimate",       throttleMuiltipliar         *   (   R    *(leftX  -   leftY   +     rightX   *  (-L)    )));
        m_telemetry.update();
    }

}
