package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotContainer.Constants;

import java.util.function.DoubleSupplier;

@Config
public class LinearSlideSubsystem extends SubsystemBase {
    private HardwareMap m_hardwareMap;
    private DcMotorEx linearSlide;
    private Telemetry m_telemetry;

    public int targetPosition;
    public String desiredLevel = "LOWLOW";

    public static double k_p = 8.0;
    public static double p = 12.0;
    public static double i = 0.0;
    public static double d = 1.0;
    public static double f = 15.0;

    public static int adjustmentLevel = 400;

    public static int zero = -15;

    public int adjustment = 0;


    public LinearSlideSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        m_hardwareMap = hardwareMap;
        m_telemetry = telemetry;

        linearSlide = m_hardwareMap.get(DcMotorEx.class, "LS");

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
        } else if(desiredLevel == "ZERO"){
            extendToZero();
        } else if(desiredLevel == "LOWLOW"){
            targetPosition = Constants.LOWLOW;
        } else if(desiredLevel == "auton"){
            targetPosition = Constants.AUTON;
        }
    }
    public void extendToZero()
    {
        targetPosition = zero;

    }
    public void resetToZero(){
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(!linearSlide.isOverCurrent()) {
            linearSlide.setPower(-0.25);
        }
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setTargetPosition(targetPosition);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void lowerSlide() {
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlide.setPower(-0.4);
    }

    public void adjustSlide(DoubleSupplier joystick){

        double power = joystick.getAsDouble();
        double m_power = 0;
        if(power > 0.7){
            adjustment -= 30;
        } else if(power < -0.7){
            adjustment += 30;
        }
    }
    public void setAdjustment(){
        adjustment += adjustmentLevel;
    }

    public void resetAdjustment(){
        adjustment = 0;
    }

//    public void setState(String state){
//        if(state == "setp"){
//            m_state = "setp";
//            linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            targetPosition = linearSlide.getCurrentPosition();
//        } else if(state == "adjust"){
//            m_state = "adjust";
//            linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }
//
//    }
    public void resetEncoder(){
        linearSlide.setPower(0.0);
        targetPosition = 0;
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        targetPosition = zero;
    }


    public void setLevel(String level){
        desiredLevel = level;
        }

    public Command changeLevelCommand(String level){
        return new InstantCommand(()-> this.setLevel(level));
    }

    public Command setAndExtendCommand(String level){
        return new SequentialCommandGroup(new InstantCommand(()-> this.setLevel(level)),
                                          new InstantCommand(()-> this.extendToTarget()));
    }

    @Override
    public void periodic(){
//        if(m_state == "setp") {
        if(targetPosition == zero){
            adjustment = 0;
            linearSlide.setTargetPosition(targetPosition);
        } else {
            linearSlide.setTargetPosition(targetPosition + adjustment);
        }



        linearSlide.setVelocityPIDFCoefficients(p, i, d, f);
        linearSlide.setPositionPIDFCoefficients(k_p);
        m_telemetry.addData("linearSlideAdjustment", adjustment);

    }
}
