package frc.robot.commands.Auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * A command group used for autonomous testing
 */
public class AutoTest extends SequentialCommandGroup{
    public AutoTest(DriveSubsystem m_drive, ShooterSubsystem m_shooter, IntakeSubsystem m_intake) {
        addCommands(
                new AutoTurnToHeading(m_drive, new Rotation2d(Math.PI / 2), 0.2)
                );
    }
}