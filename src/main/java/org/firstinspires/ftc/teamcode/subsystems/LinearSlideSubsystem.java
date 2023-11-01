package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class LinearSlideSubsystem extends SubsystemBase {
    private HardwareMap hardwareMap;
    private DcMotor linearSlide;
    private Telemetry telemetry;

    private int targetPosition;

    public LinearSlideSubsystem(HardwareMap hardwareMap,Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        linearSlide = hardwareMap.get(DcMotor.class, "LS");
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        linearSlide.setPower(1.0);
        linearSlide.setTargetPosition(0);
    }

    public void extendToTarget(int target,double power)
    {
        targetPosition = target;
        linearSlide.setTargetPosition(target);
        linearSlide.setPower(power);
    }
    public void extendToTarget(double power)
    {
        linearSlide.setTargetPosition(targetPosition);
        linearSlide.setPower(power);
    }

    public void setLevel(String level){
        if (level == "LOW") {
            targetPosition = Constants.LOW;
        } else if(level == "MEDIUM"){
            targetPosition = Constants.MEDIUM;
        } else if(level == "HIGH"){
            targetPosition = Constants.HIGH;
           }
        }


    @Override
    public void periodic(){
        linearSlide.setTargetPosition(targetPosition);
    }
}
