package org.firstinspires.ftc.teamcode.autons;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.MoveCommand;
import org.firstinspires.ftc.teamcode.subsystems.CassetSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PoseEstimationSubsystem;



public class BlueRightAuton {

    DriveSubsystem m_driveSS;
    PoseEstimationSubsystem m_poseEstimationSS;
    LinearSlideSubsystem m_linearSlideSubsystem;
    CassetSubsystem m_cassetSubsystem;
    IntakeSubsystem m_intakeSubsystem;
    //    PlaneLauncherSubsystem m_planeLauncherSubsystem;

    public BlueRightAuton(DriveSubsystem driveSubsystem,
                          PoseEstimationSubsystem poseEstimationSubsystem,
                          LinearSlideSubsystem linearSlideSubsystem,
                          CassetSubsystem cassetSubsystem,
                          IntakeSubsystem intakeSubsystem
    )
    {
        // inject subsystems and store them here
        m_driveSS = driveSubsystem;
        m_poseEstimationSS = poseEstimationSubsystem;
        m_linearSlideSubsystem = linearSlideSubsystem;
        m_cassetSubsystem = cassetSubsystem;
        m_intakeSubsystem = intakeSubsystem;
    }
    //visionSubsystem.
    public Command generate()
    {
           return new SequentialCommandGroup(
                   new WaitCommand(10)
                   .andThen(m_poseEstimationSS.resetHeadingCommand()),
                   new WaitCommand(10)
                //write your auton here as a huge command sequential command group (i.e. series of commands)
           );
    }
}
