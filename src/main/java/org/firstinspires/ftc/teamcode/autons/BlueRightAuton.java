package org.firstinspires.ftc.teamcode.autons;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.teamcode.commands.GroundDepositCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.subsystems.AprilVision2;
import org.firstinspires.ftc.teamcode.subsystems.CassetSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DoubleCassetSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PoseEstimationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class BlueRightAuton {

    DriveSubsystem m_driveSS;
    DoubleCassetSubsystem m_casset;
    IntakeSubsystem m_intake;
    LinearSlideSubsystem m_linearSlideSubsystem;
    VisionSubsystem m_vision;

    public BlueRightAuton(DriveSubsystem driveSubsystem,
                         DoubleCassetSubsystem cassetSubsystem,
                         IntakeSubsystem intakeSubsystem,
                         LinearSlideSubsystem linearSlideSubsystem,
                          VisionSubsystem visionSubsystem
    )
    {
        m_driveSS = driveSubsystem;
        m_casset = cassetSubsystem;
        m_intake = intakeSubsystem;
        m_linearSlideSubsystem = linearSlideSubsystem;
        m_vision = visionSubsystem;
    }

    public Command generate()
    {
        return new WaitCommand(0)
                .andThen(m_intake.stateChangeCommand())
                .andThen(new ConditionalCommand(
                        new TrajectoryFollowerCommand(m_driveSS, "BlueRight/BRGround1", 40, 40),
                        new WaitCommand(0),
                        (()-> m_vision.getLocation() == 0)))
                .andThen(new ConditionalCommand(
                        new TrajectoryFollowerCommand(m_driveSS, "BlueRight/BRGround2",40,40),
                        new WaitCommand(0),
                        (()-> m_vision.getLocation() == 1)))
                .andThen(new ConditionalCommand(
                        new TrajectoryFollowerCommand(m_driveSS, "BlueRight/BRGround0",40,40),
                        new WaitCommand(0),
                        (()-> m_vision.getLocation() == 2)))
                .andThen(new ConditionalCommand(
                        new ParallelCommandGroup(m_driveSS.runTrajectory("BlueRight/BRToStack1",30,30),
                        new SequentialCommandGroup(m_intake.intakeCommand(1.0),
                                m_intake.changePoseCommand(4),
                                m_intake.setStateCommand("intake"))),

                        new WaitCommand(0),
                        (()-> m_vision.getLocation() == 0)))
                .andThen(new ConditionalCommand(
                        new ParallelCommandGroup(m_driveSS.runTrajectory("BlueRight/BRToStack2",30,30),
                                new SequentialCommandGroup(m_intake.intakeCommand(1.0),
                                        m_intake.changePoseCommand(4),
                                        m_intake.setStateCommand("intake"))),
                        new WaitCommand(0),
                        (()-> m_vision.getLocation() == 1)))

                .andThen(new ConditionalCommand(

                        new ParallelCommandGroup(m_driveSS.runTrajectory("BlueRight/BRToStack0",30,30),
                                new SequentialCommandGroup(m_intake.intakeCommand(1.0),
                                        m_intake.changePoseCommand(4),
                                        m_intake.setStateCommand("intake"))),
                        new WaitCommand(0),
                        (()-> m_vision.getLocation() == 2)))

                .andThen(new WaitCommand(700))
                .andThen(m_intake.stopCommand())
                .andThen(m_intake.setStateCommand("stow"))
                .andThen(new WaitCommand(200))
                .andThen(new ConditionalCommand(
                        new ParallelCommandGroup( new TrajectoryFollowerCommand(m_driveSS, "BlueRight/BRCycleTo",40,40),
                                new SequentialCommandGroup(new WaitCommand(3500)
                                )),
                        new WaitCommand(0),
                        (()-> m_vision.getLocation() == 0)))
                .andThen(new ConditionalCommand(
                        new ParallelCommandGroup( new TrajectoryFollowerCommand(m_driveSS, "BlueRight/BRCycleTo2",40,40),
                                new SequentialCommandGroup(new WaitCommand(3500)
                                )),
                        new WaitCommand(0),
                        (()-> m_vision.getLocation() == 1)))
                .andThen(new ConditionalCommand(
                        new ParallelCommandGroup( new TrajectoryFollowerCommand(m_driveSS, "BlueRight/BRCycleTo0",40,40),
                                new SequentialCommandGroup(new WaitCommand(3500)
                                )),
                        new WaitCommand(0),
                        (()-> m_vision.getLocation() == 2)))

                .andThen(new WaitCommand(300))
                .andThen(m_linearSlideSubsystem.setAndExtendCommand("MEDIUM"))
                .andThen(new WaitCommand(700))
                .andThen(m_casset.depositBothCommand())
                .andThen(new ParallelCommandGroup( new TrajectoryFollowerCommand(m_driveSS, "BlueRight/BRCycleBack",40,40),
                        new SequentialCommandGroup(new WaitCommand(500), m_linearSlideSubsystem.setAndExtendCommand("ZERO"),
                                m_casset.intakePoseCommand(),
                                new WaitCommand(1500)
                                        .andThen(m_intake.changePoseCommand(3))
                                        .andThen(m_intake.setStateCommand("intake"))
                                        .andThen(m_intake.intakeCommand(1.0))
                        )))
                .andThen(new WaitCommand(300))
                .andThen(new ParallelCommandGroup(m_driveSS.runTrajectory("BlueRight/BShimmy"),
                        new SequentialCommandGroup(new WaitCommand(200), m_intake.changePoseCommand(2)
                        )))
                .andThen(new WaitCommand(1200))
                .andThen(m_intake.stopCommand())

                .andThen(m_intake.stateChangeCommand())
                .andThen(new ParallelCommandGroup(m_driveSS.runTrajectory("BlueRight/BRCycleTo",40,40),
                        new SequentialCommandGroup(new WaitCommand(3500))
                ))

                        .andThen(new WaitCommand(300))
                        .andThen(m_linearSlideSubsystem.setAndExtendCommand("MEDIUM"))
                .andThen(new WaitCommand(800))
                .andThen(m_casset.depositBothCommand())


                .andThen(new WaitCommand(400))
        .andThen(new ParallelCommandGroup(m_driveSS.runTrajectory("BlueRight/BParkLeft"),
                new SequentialCommandGroup(new WaitCommand(500),
                        m_linearSlideSubsystem.setAndExtendCommand("ZERO"),
                        m_casset.intakePoseCommand())));



    }
        //.andThen(new RotateCommand(45,0.2, m_driveSS));
    }
