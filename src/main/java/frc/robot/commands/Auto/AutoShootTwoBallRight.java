package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Teleop.Intake.DropIntake;
import frc.robot.commands.Teleop.Intake.StartIntake;
import frc.robot.commands.Teleop.Intake.StartTransport;
import frc.robot.commands.Teleop.Intake.StopIntake;
import frc.robot.commands.Teleop.Intake.StopTransport;
import frc.robot.commands.Teleop.Shooter.ShooterHoodDown;
import frc.robot.commands.Teleop.Shooter.StartShooter;
import frc.robot.commands.Teleop.Shooter.StopShooter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShootTwoBallRight extends SequentialCommandGroup{
    
    public AutoShootTwoBallRight(DriveSubsystem m_drive, ShooterSubsystem m_shooter, IntakeSubsystem m_intake) {
        addCommands(
            new ShooterHoodDown(m_shooter, false),
            new DropIntake(m_intake),
            new StartIntake(m_intake),
            new AutoDriveTime(m_drive, -0.4, -0.4, 2, true),
            new StartShooter(m_shooter, ShooterConstants.kShooterHighGoalCloseSpeed),
            new AutoTurnToHeading(m_drive, 180, -0.4),
            new AutoDriveTime(m_drive, -0.4, -0.4, 0.9, true),
            new WaitCommand(1),
            new StartTransport(m_shooter),
            new WaitCommand(0.5),
            new StopTransport(m_shooter),
            new WaitCommand(0.75),
            new StartTransport(m_shooter),
            new WaitCommand(2),
            new StopIntake(m_intake),
            new StopTransport(m_shooter),
            new StopShooter(m_shooter)
            );
      }
}
