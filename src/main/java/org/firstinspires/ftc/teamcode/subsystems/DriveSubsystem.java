package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * state machine for the mecanum drive. All movement/following is async to fit the paradigm.
 */
public class DriveSubsystem extends SubsystemBase {

    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry)
    {
        m_hardwareMap=hardwareMap;
        m_telemetry=telemetry;

        //TODO create all the motor objects here
    }

    public void drive(double leftX, double leftY, double rightX, double throttle, boolean isFieldCentric)
    {
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
//        );
    }

    private Vector2d quadraticControlLaw(Vector2d inputVec)
    {
        double outX = inputVec.getX() * inputVec.getX();
        double outY = inputVec.getY() * inputVec.getY();
        return new Vector2d(outX, outY);
    }

    @Override
    public void periodic()
    {
//        m_telemetry.addData("robotPosePoseEstimate", getPoseEstimate());
//        m_telemetry.update();
    }

}
