package frc.robot.commands.Autonomous.AtHomePaths.GalacticSearch;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;
import frc.robot.commands.DrivingCommands.*;
import frc.robot.commands.IntakeCommands.*;

public class PathBRed extends SequentialCommandGroup {

    DriveSubsystem m_drive;
    IntakeSubsystem m_intake;

    /**
     * Creates a new autonomous PathBRed command.
     *
     * @param driveSubsystem The drive subsystem with which this sequential command will run
     * @param intakeSubsystem The intake subsystem with which this sequential command will run
     */

    public PathBRed(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem) {
        m_drive = driveSubsystem;
        m_intake = intakeSubsystem;
        addCommands(
            new DriveDistance(m_drive,12, 1),
            new DriveDistance(m_drive,-12, 1),
            new StartIntake(m_intake),
            new TurnDegrees(m_drive, 40, .1, DriveConstants.kRight, 24),
            new DriveDistance(m_drive, 118, .3, .3),
            new TurnDegrees(m_drive, -40, .1, DriveConstants.kLeft, 0),
            new DriveDistance(m_drive, 93, .3, .3),
            new TurnDegrees(m_drive, -5, .3, DriveConstants.kRight, 81),
            new DriveDistance(m_drive, 60,1),
            new StopIntake(m_intake)
        );
    }
}
