package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.DrivingCommands.*;
import frc.robot.commands.IntakeCommands.*;

public class CompetitionAuto extends SequentialCommandGroup {

    DriveSubsystem m_drive;
    IntakeSubsystem m_intake;

    /**
     * Creates a new autonomous CompetitionAuto command.
     * This is our official autonomous robot program.
     * 
     * @param driveSubsystem the drive subsystem with which this sequential command will run
     * @param intakeSubsystem the intake subsystem with which this sequential command will run
     * 
     */

    public CompetitionAuto(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem) {
        m_drive = driveSubsystem;
        m_intake = intakeSubsystem;

        addCommands(
            new DriveDistance(m_drive, -12, 0.75, 0),
            new DriveDistance(m_drive, 36, .3, 0),
            new LaunchIntake(m_intake),
            new WaitCommand(5),
            new StopIntake(m_intake));
    }
}