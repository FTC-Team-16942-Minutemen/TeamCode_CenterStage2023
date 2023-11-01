package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.source.doctree.StartElementTree;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {
    DcMotorEx flyWheel;
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    public static double P =9;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        m_hardwareMap = hardwareMap;
        m_telemetry = telemetry;
        flyWheel = m_hardwareMap.get(DcMotorEx.class, "flyWheel");
        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void stop(){
        flyWheel.setPower(0);
        }


    public void intake(double speed){
        flyWheel.setPower(speed);


    }
    public void outake(){
        flyWheel.setPower(-1.0);


    }
    public void periodic(){
        flyWheel.setVelocityPIDFCoefficients(P, I, D, F);
        m_telemetry.addData("desired velocity", flyWheel.getVelocity());

    }

}
