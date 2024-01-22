package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.subsystems.VisionPipelines.RedDetectPipeline;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robotContainer.Constants;
import org.firstinspires.ftc.teamcode.subsystems.VisionPipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystems.VisionPipelines.BlueDetectPipeline;
import org.firstinspires.ftc.teamcode.subsystems.VisionPipelines.RedDetectPipeline;


import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.openftc.apriltag.AprilTagDetection;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


import java.util.ArrayList;


public class VisionSubsystem extends SubsystemBase {

    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    OpenCvWebcam m_webcam;

    BlueDetectPipeline m_blueImagePipeline;
    //AprilTagDetectionPipeline m_aprilTagDetection;

    Boolean m_showStage = Boolean.TRUE;
    Pose2d deposit1;
    Pose2d deposit2;
    Pose2d deposit3;
    public int pathNum = -1;
    static final double FEET_PER_METER = 3.28084;

    public VisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry)
    {

        m_hardwareMap = hardwareMap;
        m_telemetry = telemetry;
        m_blueImagePipeline = new BlueDetectPipeline();


        int cameraMonitorViewId = m_hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", m_hardwareMap.appContext.getPackageName());
        m_webcam = OpenCvCameraFactory.getInstance().createWebcam(m_hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        m_webcam.setPipeline(m_blueImagePipeline);
        m_webcam.setMillisecondsPermissionTimeout(2500);
        m_webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                                           @Override
                                           public void onOpened() {
                                               m_webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
                                           }

                                           @Override
                                           public void onError(int errorCode) {
                                               //if can't open the webcam do something here
                                           }
                                       }
        );

        //output the OpenCV processed image from the webcam to the FTCDashboard
        FtcDashboard.getInstance().startCameraStream(m_webcam, 30);
    }

    public int getLocation(){
        return m_blueImagePipeline.getLocation();
    }



    public void disablePipeline()
    {
        m_webcam.stopStreaming();
        m_showStage = Boolean.FALSE;
    }




//    detection.ftcPose.range = Math.hypot(detection.ftcPose.x, detection.ftcPose.y);
//    detection.ftcPose.bearing = outputUnitsAngle.fromUnit(AngleUnit.RADIANS, Math.atan2(-detection.ftcPose.x, detection.ftcPose.y));
//    detection.ftcPose.elevation = outputUnitsAngle.fromUnit(AngleUnit.RADIANS, Math.atan2(detection.ftcPose.z, detection.ftcPose.y));



    public void setMode(String mode) {

        if (mode == "BLUE") {
            m_blueImagePipeline.H_start = Constants.H_START_BLUE;
            m_blueImagePipeline.H_end = Constants.H_END_BLUE;
        } else if (mode == "RED") {
            m_blueImagePipeline.H_start = Constants.H_START_RED;
            m_blueImagePipeline.H_end = Constants.H_END_RED;

        }
    }

    @Override
    public void periodic() {
        if(m_showStage) {
            m_telemetry.addData("Detected Spot:", m_blueImagePipeline.getLocation());
            m_telemetry.update();
        }
    }


}

