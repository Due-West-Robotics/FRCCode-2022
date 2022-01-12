package frc.robot.commands.Autonomous.AtHomePaths.GalacticSearch;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;
import frc.robot.commands.DrivingCommands.*;
import frc.robot.commands.IntakeCommands.*;

public class PathABlue extends SequentialCommandGroup {

    DriveSubsystem m_drive;
    IntakeSubsystem m_intake;

    /**
     * Creates a new autonomous PathABlue command.
     *
     * @param driveSubsystem The drive subsystem with which this sequential command will run
     * @param intakeSubsystem The intake subsystem with which this sequential command will run
     */

    public PathABlue(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem) {
        m_drive = driveSubsystem;
        m_intake = intakeSubsystem;
        addCommands(
            new DriveDistance(m_drive,120, 0.5),
            new DriveDistance(m_drive,-24, 0.65),
            new StartIntake(m_intake),
            new DriveDistance(m_drive,54, 0.35),
            new TurnDegrees(m_drive, -65, .1, DriveConstants.kLeft, 0),
            new DriveDistance(m_drive, 108, 0.35),
            new TurnDegrees(m_drive, 25, .1, DriveConstants.kRight, 0),
            new DriveDistance(m_drive, 48, 0.5),
            new TurnDegrees(m_drive, 10, .1, DriveConstants.kLeft, 15),
            new DriveDistance(m_drive, 60, 1)
        );
    }
}