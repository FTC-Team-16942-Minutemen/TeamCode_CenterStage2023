package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PoseEstimationSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private final DriveSubsystem m_driveSubsystem;
    private final DoubleSupplier m_leftYSupplier;
    private final DoubleSupplier m_leftXSupplier;
    private final DoubleSupplier m_rightXSupplier;
    private final DoubleSupplier m_rightTriggerSupplier;
    private final PoseEstimationSubsystem m_poseEstimationSubsystem;
    private boolean m_isFieldCentric;

    public DriveCommand(DriveSubsystem drive,
                        PoseEstimationSubsystem poseEstimationSubsystem,
                        DoubleSupplier leftYSupplier,
                        DoubleSupplier leftXSupplier,
                        DoubleSupplier rightXSupplier,
                        DoubleSupplier rightTriggerSupplier,
                        boolean isFieldCentric) {

        m_driveSubsystem = drive;
        m_poseEstimationSubsystem = poseEstimationSubsystem;
        m_leftXSupplier = leftXSupplier;
        m_leftYSupplier = leftYSupplier;
        m_rightXSupplier = rightXSupplier;
        m_rightTriggerSupplier = rightTriggerSupplier;
        m_isFieldCentric = isFieldCentric;

        addRequirements(m_driveSubsystem);
        addRequirements(m_poseEstimationSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_driveSubsystem.drive(m_leftXSupplier.getAsDouble(),
                m_leftYSupplier.getAsDouble(),
                m_rightXSupplier.getAsDouble(),
                m_rightTriggerSupplier.getAsDouble(),
                m_isFieldCentric ? m_poseEstimationSubsystem.getPose().getHeading():0.0);
    }

}