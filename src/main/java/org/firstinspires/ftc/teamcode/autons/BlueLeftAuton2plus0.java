
package org.firstinspires.ftc.teamcode.autons;

import com.arcrobotics.ftclib.command.Command;

import com.arcrobotics.ftclib.command.ConditionalCommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;


import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.teamcode.commands.GroundDepositCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;

import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AlignmentScoringCommand;
import org.firstinspires.ftc.teamcode.commands.GroundDepositCommand;
import org.firstinspires.ftc.teamcode.commands.RelocalizeOffAprilTags;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.subsystems.AprilVision;
import org.firstinspires.ftc.teamcode.subsystems.AprilVision2;
import org.firstinspires.ftc.teamcode.subsystems.DoubleCassetSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PoseEstimationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;




public class BlueLeftAuton2plus0 {

    DriveSubsystem m_driveSS;
    DoubleCassetSubsystem m_casset;
    IntakeSubsystem m_intake;
    LinearSlideSubsystem m_linearSlideSubsystem;
    VisionSubsystem m_vision;
    Telemetry m_telemetry;


    public BlueLeftAuton2plus0(DriveSubsystem driveSubsystem,
                               DoubleCassetSubsystem cassetSubsystem,
                               IntakeSubsystem intakeSubsystem,
                               LinearSlideSubsystem linearSlideSubsystem,
                               VisionSubsystem vision
    )
    {
        m_driveSS = driveSubsystem;
        m_casset = cassetSubsystem;
        m_intake = intakeSubsystem;
        m_linearSlideSubsystem = linearSlideSubsystem;
        m_vision = vision;

    }

    public Command generate()
    {
        // return new WaitCommand(0).andThen(m_driveSS.runTrajectory("BlueLeft/StartTuner"));

        return new WaitCommand(0)
                .andThen(m_intake.stateChangeCommand())
                .andThen(new ConditionalCommand(
                        new TrajectoryFollowerCommand(m_driveSS, "BlueLeft/BLGround0", 45, 45),
                        new WaitCommand(0),
                        (()-> m_vision.getLocation() == 0)))
                .andThen(new ConditionalCommand(
                        new TrajectoryFollowerCommand(m_driveSS, "BlueLeft/BLGround1",45,45),
                        new WaitCommand(0),
                        (()-> m_vision.getLocation() == 1)))
                .andThen(new ConditionalCommand(
                        new TrajectoryFollowerCommand(m_driveSS, "BlueLeft/BLGround2",45,45),
                        new WaitCommand(0),
                        (()-> m_vision.getLocation() == 2)))
                .andThen(m_intake.changePoseCommand(0))
                .andThen(new WaitCommand(200))
                .andThen(m_intake.intakeCommand(-0.5))
                .andThen(new WaitCommand(800))
                .andThen(m_intake.stopCommand())

                .andThen(m_intake.stateChangeCommand())
                .andThen(new ConditionalCommand(
                        m_driveSS.runTrajectory("BlueLeft/BLDeposit0",45,45),
                        new WaitCommand(0),
                        (()-> m_vision.getLocation() == 0)))
                .andThen(new ConditionalCommand(
                        m_driveSS.runTrajectory("BlueLeft/BLDeposit1",45,45),
                        new WaitCommand(0),
                        (()-> m_vision.getLocation() == 1)))
                .andThen(new ConditionalCommand(
                        m_driveSS.runTrajectory("BlueLeft/BLDeposit2",45,45),
                        new WaitCommand(0),
                        (()-> m_vision.getLocation() == 2)))
                .andThen(m_linearSlideSubsystem.setAndExtendCommand("LOWLOW"))
                .andThen(new WaitCommand(500))
                .andThen(m_casset.depositBothCommand())
                .andThen(new WaitCommand(200))

                .andThen(new ParallelCommandGroup(m_driveSS.runTrajectory("BlueLeft/BParkLeft"),
                        new SequentialCommandGroup(new WaitCommand(500),
                                m_linearSlideSubsystem.setAndExtendCommand("ZERO"),
                                m_casset.intakePoseCommand())));



    }
}