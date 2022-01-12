package frc.robot.commands.Autonomous.AtHomePaths.GalacticSearch;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;
import frc.robot.commands.DrivingCommands.*;
import frc.robot.commands.IntakeCommands.*;

public class PathBBlue extends SequentialCommandGroup {

    DriveSubsystem m_drive;
    IntakeSubsystem m_intake;

    /**
     * Creates a new autonomous PathBBlue command.
     *
     * @param driveSubsystem The drive subsystem with which this sequential command will run
     * @param intakeSubsystem The intake subsystem with which this sequential command will run
     */

    public PathBBlue(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem) {
        m_drive = driveSubsystem;
        m_intake = intakeSubsystem;
        addCommands(
            new DriveDistance(m_drive,12, 1),
            new DriveDistance(m_drive,-12, 1),
            new StartIntake(m_intake),
            new TurnDegrees(m_drive, -6, 0.1 , DriveConstants.kLeft, 0),
            new DriveDistance(m_drive, 144, .3),
            new TurnDegrees(m_drive, -40,.1, DriveConstants.kLeft, 0),
            new DriveDistance(m_drive, 96, .3),
            new TurnDegrees(m_drive, 54, .1, DriveConstants.kRight, 0),
            new DriveDistance(m_drive, 96, .3, .3),
            new TurnDegrees(m_drive, 5, .3, DriveConstants.kLeft, 48),
            new DriveDistance(m_drive, 48, 1)
        );
    }
}