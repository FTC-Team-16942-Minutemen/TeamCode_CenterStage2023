package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autons.BlueRightAuton;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
//import org.firstinspires.ftc.teamcode.commands.MoveCommand;
import org.firstinspires.ftc.teamcode.commands.MoveCommand;
import org.firstinspires.ftc.teamcode.subsystems.CassetSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PlaneLauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PoseEstimationSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Config
public class RobotContainer extends Robot {
    //Basic hardware components
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    GamepadEx m_gamePad1;
    GamepadEx m_gamePad2;

    //Subsystems
    DriveSubsystem m_driveSS;
    PoseEstimationSubsystem m_poseEstimationSS;
    LinearSlideSubsystem m_linearSlideSubsystem;
    CassetSubsystem m_cassetSubsystem;
     IntakeSubsystem m_intakeSubsystem;
     PlaneLauncherSubsystem m_planeLauncherSubsystem;
 //    PlaneLauncherSubsystem m_planeLauncherSubsystem;



     public static double defaultSpeed = 0.9;

    public RobotContainer(Constants.OpModeType opModetype,
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
        m_driveSS = new DriveSubsystem(m_hardwareMap, m_telemetry);
        m_poseEstimationSS = new PoseEstimationSubsystem(m_hardwareMap, m_telemetry);

        m_poseEstimationSS.reset(0,0);

        m_cassetSubsystem = new CassetSubsystem(m_hardwareMap, m_telemetry,0.97);
        m_planeLauncherSubsystem = new PlaneLauncherSubsystem(m_hardwareMap, m_telemetry, 0);
        m_linearSlideSubsystem = new LinearSlideSubsystem(m_hardwareMap,m_telemetry);
        m_linearSlideSubsystem.reset();
          m_intakeSubsystem = new IntakeSubsystem(m_hardwareMap, m_telemetry);

        switch(opModetype) {
            case TELEOP:
                m_telemetry.addData("Initialize","TeleOp");
                setupTeleOp();
                break;
            case BLUE_LEFT_AUTO:
                m_telemetry.addData("Initialize","Blue Left Auton");
                setupBlueLeft_Auton();
                break;
            case BLUE_RIGHT_AUTO:
                m_telemetry.addData("Initialize","Blue Right Auton");
                setupBlueRight_Auton();
                break;
            case RED_LEFT_AUTO:
                m_telemetry.addData("Initialize","Red Left Auton");
                setupRedLeft_Auton();
                break;
            case RED_RIGHT_AUTO:
                m_telemetry.addData("Initialize","Red Right Auton");
                setupRedRight_Auton();
                break;
            default:
                m_telemetry.addData("Unknown type", opModetype);
                break;
        }
        m_telemetry.update();
    }

//    public void disableVision() //TODO determine what to do with these methods
//    {
//        m_visionSubsystem.disablePipeline();
//    }

//    public void setRobotPose(Pose2d inputPose)
//    {
//        m_driveTrain.setPoseEstimate(inputPose);
//    }

    private void setupTeleOp()
    {
        //Set motor attributes for TeleOp
//        m_driveSS.setMotorZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // TODO: make sure BRAKE is best for Teleop
        m_driveSS.setMotorMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        m_driveSS.setDefaultCommand(new DriveCommand(m_driveSS,
                m_poseEstimationSS,
                ()->m_gamePad1.getLeftY(),
                ()->-m_gamePad1.getLeftX(),
                ()->-m_gamePad1.getRightX(),
                ()->m_gamePad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
                Constants.FIELD_CENTRIC_ENABLED));

        //Gamepad Button (Trigger) mappings
//
//        m_gamePad1.getGamepadButton(GamepadKeys.Button.Y)
//                .whileHeld(new InstantCommand(() -> {m_int
//                akeSubsystem.intake(Constants.DEFAULT_INTAKE_SPEED);}));

        m_gamePad2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(
                        new InstantCommand(() -> {m_cassetSubsystem.depositPosition();})
                );


        m_gamePad2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> {m_cassetSubsystem.intakePosition();}));

        m_gamePad2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> {m_planeLauncherSubsystem.shoot();}));


        m_gamePad2.getGamepadButton(GamepadKeys.Button.X)
                        .whileHeld(new InstantCommand(() -> {m_intakeSubsystem.intake(1.0);}))
                        .whenReleased(new InstantCommand(()-> {m_intakeSubsystem.stop();}));

        m_gamePad2.getGamepadButton(GamepadKeys.Button.Y)
                .whileHeld(new InstantCommand(() -> {m_intakeSubsystem.outake();}))
                .whenReleased(new InstantCommand(()-> {m_intakeSubsystem.stop();}));


        m_gamePad2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.setLevel("HIGH");}));
//
//
        m_gamePad2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.setLevel("LOW");}));

        m_gamePad2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.setLevel("MEDIUM");}));

        m_gamePad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.extendToZero();}));
//
//
        m_gamePad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> {m_linearSlideSubsystem.extendToTarget();}));

        m_gamePad1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(m_poseEstimationSS.resetHeadingCommand());


//
//        m_gamePad1.getGamepadButton(GamepadKeys.Button.X)
//                .whenPressed(new MoveCommand(0,0.30159289474,0.7, m_driveSS));
//
//        m_gamePad1.getGamepadButton(GamepadKeys.Button.B)
//                .whenPressed(new MoveCommand(0,-0.30159289474,0.7, m_driveSS));

//        m_gamePad1.getGamepadButton(GamepadKeys.Button.Y)
//                .whenPressed(new MoveCommand(m_driveSS, m_poseEstimationSS, 0,5,0));

//        m_gamePad1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                .whenPressed(new InstantCommand(() -> {m_turntableSubsystem.faceForward();}));
//        m_gamePad1.getGamepadButton(GamepadKeys.Button.A)
//                .whenPressed(new InstantCommand(() -> {m_driveSS.setPoseEstimate(m_distancelocalizerSubsystem.RelocalizeWestWall());}));

    }

    private void setupBlueRight_Auton()
    {
        BlueRightAuton blueRightAuton = new BlueRightAuton(m_driveSS,
                                                              m_poseEstimationSS,
                                                               m_linearSlideSubsystem,
                                                                           m_cassetSubsystem,
                                                                m_intakeSubsystem); //TODO add the injected subsystems here
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