package frc.robot.commands.Autonomous.AtHomePaths.GalacticSearch;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;
import frc.robot.commands.DrivingCommands.*;
import frc.robot.commands.IntakeCommands.*;

public class PathARed extends SequentialCommandGroup {

    DriveSubsystem m_drive;
    IntakeSubsystem m_intake;

    /**
     * Creates a new autonomous PathARed command.
     *
     * @param driveSubsystem The drive subsystem with which this sequential command will run
     * @param intakeSubsystem The intake subsystem with which this sequential command will run
     */

    public PathARed(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem) {
        m_drive = driveSubsystem;
        m_intake = intakeSubsystem;
        addCommands(
            new DriveDistance(m_drive,24, 1),
            new DriveDistance(m_drive,-24, 1),
            new StartIntake(m_intake),
            new DriveDistance(m_drive,72, 0.2),//originally 94
            new TurnDegrees(m_drive, 65.374, 0.2, DriveConstants.kRight, 21.176),
            new TurnDegrees(m_drive, 245 + 65.374, 0.2, DriveConstants.kRight, 11),
            new DriveDistance(m_drive, 93.675, 0.2),
            new TurnDegrees(m_drive, 350, 0.2, DriveConstants.kRight, 0),
            new DriveDistance(m_drive, 155, 1)
        );
    }
}