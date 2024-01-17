package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class PlaneLauncherSubsystem extends SubsystemBase {
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    Servo m_launcherServo;
    double m_position;

    public static double minScale = 0.00;
    public static double maxScale = 1.00;

    public static double upPose = 0.5;
    public static double downPose = 0.0;

    public PlaneLauncherSubsystem(HardwareMap hardwareMap, Telemetry telemetry, double initial_position) {
        m_hardwareMap = hardwareMap;
        m_telemetry = telemetry;
        m_launcherServo = hardwareMap.get(Servo.class, "airplane");
        m_position = downPose;
    }

    public void shoot() {
        m_position = upPose;
    }

    public void reload() {
        m_position = downPose;
    }

    public void actuate() {
        m_position = (m_position + 1.0) % 2.0;
    }

    public double getCassetPosition() {
        return m_position;
    }

    public Command shootCommand() {
        return new InstantCommand(() -> this.shoot());
    }

    @Override
    public void periodic() {
        m_launcherServo.setPosition(m_position);
        m_launcherServo.scaleRange(minScale, maxScale);

        m_telemetry.addData("launcher Pos: ", m_launcherServo.getPosition());
        m_telemetry.addData("Set Position: ", m_position);
        m_telemetry.update();
    }
}
