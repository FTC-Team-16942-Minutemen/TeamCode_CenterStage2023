package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AlignmentCommand;
import org.firstinspires.ftc.teamcode.commands.AutoTargetingDriveCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.ParkingCommand;
import org.firstinspires.ftc.teamcode.commands.ScoringCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.TurntableTurnCommand;
import org.firstinspires.ftc.teamcode.robots.triggers.LeftTriggerTrigger;
import org.firstinspires.ftc.teamcode.robots.triggers.TurntableTrigger;
import org.firstinspires.ftc.teamcode.subsystems.ClawIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.AlignmentSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DistancelocalizerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurntableSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.Constants;
import java.io.IOException;
import java.util.function.BooleanSupplier;

@Config
public class RobotContainer extends Robot {
    //Basic hardware components
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    GamepadEx m_gamePad1;
    GamepadEx m_gamePad2;

    public PowerPlayBot(Constants.OpModeType type,
                     HardwareMap hardwareMap,
                     Telemetry telemetry,
                     Gamepad gamePad1,
                        Gamepad gamePad2,
                        Pose2d initialPose,
                        double allianceHeadingOffset)
    {

        setupOpMode(type);
    }

    public void disableVision()
    {
        m_visionSubsystem.disablePipeline();
    }

//    public void setCurrentTime(double time)
//    {
//        m_timedParkingTrigger.setTime(time);
//    }

    public Pose2d getRobotPose()
    {
        return m_driveTrain.getPoseEstimate();
    }

//    public void setRobotPose(Pose2d inputPose)
//    {
//        m_driveTrain.setPoseEstimate(inputPose);
//    }

    private void setupOpMode(Constants.OpModeType type)
    {
        if(type == Constants.OpModeType.TELEOP)
        {
            m_telemetry.addData("Initialize","TeleOp");
            setupTeleOp();
        }
        else if (type == Constants.OpModeType.BLUE_RIGHT_AUTO)
        {

            m_telemetry.addData("Initialize", "BlueRight_Auton");
            setupBlueRight_Auton();
        }
        else if (type == Constants.OpModeType.BLUE_LEFT_AUTO)
        {
            m_telemetry.addData("Initialize", "BlueLeft_Auton");
            setupBlueLeft_Auton();
        }
        else if (type == Constants.OpModeType.RED_RIGHT_AUTO)
        {
            m_telemetry.addData("Initialize", "RedRight_Auton");
            setupRedRight_Auton();
        }
        else if (type == Constants.OpModeType.RED_LEFT_AUTO)
        {
            m_telemetry.addData("Initialize", "RedLeft_Auton");
            setupRedLeft_Auton();
        }

        m_telemetry.update();
    }

    private void setupTeleOp() {}
        /*
//        m_leftTriggerTrigger.whileActiveOnce(new ScoringCommand(m_clawIntakeSubsystem,
//                m_linearSlideSubsystem,
//                ()->m_gamePad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)))
//                .whenInactive(new SequentialCommandGroup(
//                            new WaitCommand(100),
//                            new InstantCommand(() -> {m_linearSlideSubsystem.setElevatorPower(1.0);})
//                        )
//                );
//        m_leftTriggerTrigger.whileActiveOnce(new InstantCommand())
//        m_turntableTrigger.whileActiveOnce(
//                new SequentialCommandGroup(
//                        new WaitCommand(500),
//                        new InstantCommand(()-> {m_turntableSubsystem.depositPosition();}))
//                        ).whenInactive(
//                        new InstantCommand(()-> {m_turntableSubsystem.intakePosition();}));
        m_leftTriggerTrigger.whileActiveContinuous(new AutoTargetingDriveCommand(m_driveTrain,
                m_visionSubsystem,
                ()->m_gamePad1.getLeftY(),
                ()->-m_gamePad1.getLeftX(),
                ()->-m_gamePad1.getRightX(),
                false));





        m_driveTrain.setDefaultCommand(new DriveCommand(m_driveTrain,
                ()->m_gamePad1.getLeftY(),
                ()->-m_gamePad1.getLeftX(),
                ()->-m_gamePad1.getRightX(),
                ()->m_gamePad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
                true));

        m_gamePad1.getGamepadButton(GamepadKeys.Button.Y)
                        .whenPressed(new InstantCommand(() -> {m_clawIntakeSubsystem.actuate();}));

        m_gamePad1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> {m_turntableSubsystem.actuate();}));

//        m_gamePad1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                .whenPressed(new InstantCommand(() -> {m_turntableSubsystem.faceForward();}));
//        m_gamePad1.getGamepadButton(GamepadKeys.Button.A)
//                .whenPressed(new InstantCommand(() -> {m_driveTrain.setPoseEstimate(m_distancelocalizerSubsystem.RelocalizeWestWall());}));

        m_gamePad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new SequentialCommandGroup(
                                new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.JUNCTIONLEVEL);}),
                                new ConditionalCommand(
                                        new WaitCommand(0),
                                        new SequentialCommandGroup(
                                                new WaitCommand(550),
                                                new InstantCommand(m_turntableSubsystem::depositPosition, m_turntableSubsystem)
                                        ),
                                        () -> m_turntableSubsystem.getTurntablePosition() == 0.0
                                )
                        )
                );

        m_gamePad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new SequentialCommandGroup(
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new InstantCommand(m_turntableSubsystem::intakePosition, m_turntableSubsystem),
                                                new WaitCommand(550)
                                        ),
                                        new WaitCommand(0),
                                        () -> m_turntableSubsystem.getTurntablePosition() == 0.0
                                ),
                                new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.STACKLEVEL);})
                        )
                );

        m_gamePad1.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new SequentialCommandGroup(
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new InstantCommand(m_turntableSubsystem::intakePosition, m_turntableSubsystem),
                                                new WaitCommand(550)
                                        ),
                                        new WaitCommand(0),
                                        () -> m_turntableSubsystem.getTurntablePosition() == 0.0
                                ),
                                new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.GROUNDLEVEL);})
                        )
                );
//hello
        m_gamePad1.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(() -> {m_clawIntakeSubsystem.close();}),
                        new WaitCommand(250),
                        new InstantCommand(() -> {m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.JUNCTIONLEVEL);}),
                        new ConditionalCommand(
                                new WaitCommand(0),
                                new SequentialCommandGroup(
                                        new WaitCommand(550),
                                        new InstantCommand(m_turntableSubsystem::depositPosition, m_turntableSubsystem)
                                ),
                                () -> m_turntableSubsystem.getTurntablePosition() == 0.0
                        )
                        ));

        m_gamePad1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(() -> {m_linearSlideSubsystem.setBeaconCap();}),
                        new WaitCommand(1000),
                        new InstantCommand(() -> {m_clawIntakeSubsystem.open();})));
//        m_gamePad1.getGamepadButton(GamepadKeys.Button.A)
//                        .whenHeld(new AlignmentCommand(m_AlignmentSubsystem, m_driveTrain,
//                                ()->m_gamePad1.getLeftY(),
//                                ()->-m_gamePad1.getLeftX(),
//                                ()->-m_gamePad1.getRightX(),
//                                ()->m_gamePad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
//                                true));

        m_gamePad2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.setJunctionLevel(2);}));
        m_gamePad2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.setJunctionLevel(0);}));
        m_gamePad2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.setJunctionLevel(1);}));

//        m_gamePad2.getGamepadButton(GamepadKeys.Button.BACK)
//                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.toggleOperatorMode();}));
//        m_gamePad2.getGamepadButton(GamepadKeys.Button.BACK)
//                        .whenPressed(new InstantCommand(() -> {m_driveTrain.TogglePotentialFields();}));

        m_gamePad2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.setStackLevel(3);}));
        m_gamePad2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.setStackLevel(2);}));
        m_gamePad2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.setStackLevel(1);}));
        m_gamePad2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.setStackLevel(0);}));

        m_gamePad2.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenHeld(new InstantCommand(() -> {m_linearSlideSubsystem.lowerSlide();}))
                .whenReleased(new InstantCommand(() -> {m_linearSlideSubsystem.resetEncoder();}));

        m_gamePad2.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(() -> {m_driveTrain.correctHeadingOffset();}));
    }
   */

        private void setupBlueRight_Auton ()
        {
        }
        private void setupBlueLeft_Auton () {
        }

        private void setupRedRight_Auton ()
        {
        }


        private void setupRedLeft_Auton ()
        {
        }



}

