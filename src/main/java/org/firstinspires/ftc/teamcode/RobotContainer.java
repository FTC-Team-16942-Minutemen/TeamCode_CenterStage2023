package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autons.BlueRightAuton;

@Config
public class RobotContainer extends Robot {
    //Basic hardware components
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    GamepadEx m_gamePad1;
    GamepadEx m_gamePad2;

    //Subsystems
//    DriveSubsystem m_driveTrain; // TODO make a Drivetrain Subsystem

    public RobotContainer(Constants.OpModeType type,
                     HardwareMap hardwareMap,
                     Telemetry telemetry,
                     Gamepad gamePad1,
                     Gamepad gamePad2)
    {
        //Initialize basic hardware structures
        m_hardwareMap = hardwareMap;
        m_gamePad1 = new GamepadEx(gamePad1);
        m_gamePad2 = new GamepadEx(gamePad2);
        m_telemetry = telemetry;

        //Setup the FTC dashboard with it's enhanced telemetry
        m_telemetry = new MultipleTelemetry(m_telemetry, FtcDashboard.getInstance().getTelemetry());

        //Initialize Subsystems
//        m_driveTrain = new DriveSubsystem(m_hardwareMap, m_telemetry, initialPose, allianceHeadingOffset);

        setupOpMode(type);
    }

//    public void disableVision() //TODO might need to remove these next two methods
//    {
//        m_visionSubsystem.disablePipeline();
//    }

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

    private void setupTeleOp()
    {

//        m_driveTrain.setDefaultCommand(new DriveCommand(m_driveTrain,
//                ()->m_gamePad1.getLeftY(),
//                ()->-m_gamePad1.getLeftX(),
//                ()->-m_gamePad1.getRightX(),
//                ()->m_gamePad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
//                Constants));

//        m_gamePad1.getGamepadButton(GamepadKeys.Button.Y)
//                        .whenPressed(new InstantCommand(() -> {m_clawIntakeSubsystem.actuate();}));

//        m_gamePad1.getGamepadButton(GamepadKeys.Button.B)
//                .whenPressed(new InstantCommand(() -> {m_turntableSubsystem.actuate();}));

//        m_gamePad1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                .whenPressed(new InstantCommand(() -> {m_turntableSubsystem.faceForward();}));
//        m_gamePad1.getGamepadButton(GamepadKeys.Button.A)
//                .whenPressed(new InstantCommand(() -> {m_driveTrain.setPoseEstimate(m_distancelocalizerSubsystem.RelocalizeWestWall());}));

    }

    private void setupBlueRight_Auton()
    {
        BlueRightAuton blueRightAuton = new BlueRightAuton(); //TODO add the injected subsystems here
        CommandScheduler.getInstance().schedule(blueRightAuton.generate());
    }

    private void setupBlueLeft_Auton()
    {
//        BlueLeftAuton blueLeftAuton = new BlueLeftAuton();
//        CommandScheduler.getInstance().schedule(blueLeftAuton.generate());
    }

    private void setupRedRight_Auton()
    {
//        RedRightAuton redRightAuton = new RedRightAuton();
//        CommandScheduler.getInstance().schedule(redRightAuton.generate());
    }

    private void setupRedLeft_Auton()
    {
//        RedLeftAuton redLeftAuton = new RedLeftAuton();
//        CommandScheduler.getInstance().schedule(redLeftAuton.generate());
    }
}

