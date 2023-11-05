package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.visionpipelines.BlueDetectPipeline;
import org.firstinspires.ftc.teamcode.subsystems.visionpipelines.RedDetectPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class VisionSubsystem extends SubsystemBase {

    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    OpenCvWebcam m_webcam;
    RedDetectPipeline m_redImagePipeline;
    Boolean m_showStage = Boolean.TRUE;
    Pose2d deposit1;
    Pose2d deposit2;
    Pose2d deposit3;


    public VisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry)
    {
        m_hardwareMap = hardwareMap;
        m_telemetry = telemetry;
        m_redImagePipeline = new RedDetectPipeline();

        int cameraMonitorViewId = m_hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", m_hardwareMap.appContext.getPackageName());
        m_webcam = OpenCvCameraFactory.getInstance().createWebcam(m_hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        m_webcam.setPipeline(m_redImagePipeline);
        m_webcam.setMillisecondsPermissionTimeout(2500);
        m_webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                                           @Override
                                           public void onOpened() {
                                               m_webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
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

    @Override
    public void periodic()

    {
        m_telemetry.addData("Location", getLocation());
        if(m_showStage) {
//            m_telemetry.addData("Location:", m_redImagePipeline.getLocation());
            m_telemetry.update();
        }
    }

    public void disablePipeline()
    {
        m_webcam.stopStreaming();
        m_showStage = Boolean.FALSE;
    }

    public Pose2d getLocation(){

            if (m_redImagePipeline.getLocation() == 0)//magenta
            {
                return deposit1;
            }
            else if (m_redImagePipeline.getLocation() == 1) //green
            {
                return deposit2;            }
            else if (m_redImagePipeline.getLocation() == 2) //cyan
            {
                return deposit3;            }
            return deposit1;
    }




}
