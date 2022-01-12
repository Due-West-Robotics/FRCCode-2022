package frc.robot.commands.Autonomous.AtHomePaths.AutoNav;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.*;
import frc.robot.commands.DrivingCommands.*;

public class BouncePath extends SequentialCommandGroup {

    DriveSubsystem m_drive;

    /**
     * Creates a new autonomous BouncePath command.
     *
     * @param driveSubsystem The drive subsystem with which this sequential command will run
     */

    public BouncePath(DriveSubsystem driveSubsystem) {
        m_drive = driveSubsystem;
        addCommands(
            //to first point
            new DriveDistance(m_drive,19, 1),
            new TurnDegrees(m_drive, -87, .2, DriveConstants.kLeft, 30),
            new DriveDistance(m_drive, 30, 1),
            //to second point
            new DriveDistance(m_drive,-30, 1),
            new TurnDegrees(m_drive, -115, -.2, DriveConstants.kRight, 27),
            new DriveDistance(m_drive,-72, 1),
            new TurnDegrees(m_drive, -265, -.2, DriveConstants.kRight, 29.5),
            new DriveDistance(m_drive, -90, 1),
            //to third point
            new DriveDistance(m_drive, 90, 1),
            new TurnDegrees(m_drive, -356, .2, DriveConstants.kLeft, 27),
            new DriveDistance(m_drive,45, 1),
            new TurnDegrees(m_drive, -445, .2, DriveConstants.kLeft, 27),
            new DriveDistance(m_drive,95, 1),
            //to end
            new TurnDegrees(m_drive, -520, -.2, DriveConstants.kRight, 27),
            new DriveDistance(m_drive, -12, 1));
    }
}