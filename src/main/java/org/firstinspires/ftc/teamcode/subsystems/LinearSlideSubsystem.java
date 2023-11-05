package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class LinearSlideSubsystem extends SubsystemBase {
    private HardwareMap hardwareMap;
    private DcMotorEx linearSlide;
    private Telemetry telemetry;

    private int targetPosition;
    public String desiredLevel = "";





    public LinearSlideSubsystem(HardwareMap hardwareMap,Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        linearSlide = hardwareMap.get(DcMotorEx.class, "LS");

        MotorConfigurationType motorConfigurationType = linearSlide.getMotorType();
        motorConfigurationType.setAchieveableMaxRPMFraction(0.99);
        motorConfigurationType.setMaxRPM(435);

        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setTargetPosition(0);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(0.5);


    }
    public void reset(){
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setTargetPosition(0);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void extendToTarget()
    {
        if(desiredLevel == "MEDIUM"){
            targetPosition = Constants.MEDIUM;
        } else if(desiredLevel == "HIGH"){
            targetPosition = Constants.HIGH;
        } else if (desiredLevel == "LOW") {
            targetPosition = Constants.LOW;
        }
    }
    public void extendToZero()
    {
        targetPosition = 0;

    }

    public void setLevel(String level){
        desiredLevel = level;
        }


    @Override
    public void periodic(){

        linearSlide.setTargetPosition(targetPosition);


    }
}
