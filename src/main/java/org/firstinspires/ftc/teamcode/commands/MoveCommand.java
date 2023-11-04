package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PoseEstimationSubsystem;

public class MoveCommand extends CommandBase {


    private final DriveSubsystem m_driveSubsystem;
    private final PoseEstimationSubsystem m_poseEstimationSubsystem;
    private double m_goX;
    private double m_goY;
    private double m_angle;
    PIDController x;
    PIDController y;
    PIDController heading;
    Pose2d m_pose;

    public MoveCommand(DriveSubsystem driveSubsystem,
                       PoseEstimationSubsystem poseEstimationSubsystem,
                       double goX, double goY,
                       double angle) {

        m_driveSubsystem = driveSubsystem;
        m_poseEstimationSubsystem = poseEstimationSubsystem;
        m_goX = goX;
        m_goY = goY;
        m_angle = angle;
    }

    @Override
    public void initialize() {


        m_driveSubsystem.setMotorPowers(0.05);
        x = new PIDController(5,0,0);
        y = new PIDController(5,0,0);
        heading = new PIDController(5,0,0);


    }
        @Override
    public void execute(){
         m_pose = m_poseEstimationSubsystem.positionEstimation();


        m_driveSubsystem.drive(x.calculate(m_pose.getX() ,m_goX),
                y.calculate(m_pose.getY(), m_goY),
                heading.calculate(m_pose.getHeading(), m_angle), m_pose.getHeading());



        }
        @Override
    public boolean isFinished(){

        return !m_driveSubsystem.isBusy();
    }
    @Override
    public void end(boolean interupted){

        m_driveSubsystem.setMotorPowers(0);
    }
}
