package frc.robot.commands.Auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Teleop.Intake.StartTransport;
import frc.robot.commands.Teleop.Intake.StopTransport;
import frc.robot.commands.Teleop.Shooter.ShooterHoodDown;
import frc.robot.commands.Teleop.Shooter.StartShooter;
import frc.robot.commands.Teleop.Shooter.StopShooter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoTest extends SequentialCommandGroup{
    
    public AutoTest(DriveSubsystem m_drive, ShooterSubsystem m_shooter, IntakeSubsystem m_intake) {
        addCommands(
            new TurnToHeading(m_drive, new Rotation2d(Math.PI / 2), 0.2)
        );
      }
}
