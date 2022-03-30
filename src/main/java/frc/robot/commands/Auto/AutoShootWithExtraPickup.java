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

public class AutoShootWithExtraPickup extends SequentialCommandGroup{
    
    public AutoShootWithExtraPickup(DriveSubsystem m_drive, ShooterSubsystem m_shooter, IntakeSubsystem m_intake) {
        addCommands(
            new ShooterHoodDown(m_shooter, false),
            new StartShooter(m_shooter, ShooterConstants.kShooterHighGoalCloseSpeed),
            new AutoDriveTime(m_drive, 0.3, 0.3, 0.65, true),
            new WaitCommand(2),
            new StartTransport(m_shooter),
            new WaitCommand(3),  // Transport time
            new StopShooter(m_shooter),
            new StopTransport(m_shooter),
            new AutoDriveTime(m_drive, -0.3, 0.3, 1, true),
            new DropIntake(m_intake),
            new WaitCommand(1),
            new StartIntake(m_intake),
            new AutoDriveTime(m_drive, -0.2, -0.2, 2, true),
            new StopIntake(m_intake),
            new AutoDriveTime(m_drive, 0.3, -0.3, 1, true),
            new AutoDriveTime(m_drive, -0.2, -0.2, 2, true),
            new StartShooter(m_shooter, ShooterConstants.kShooterHighGoalCloseSpeed),
            new WaitCommand(1),
            new StartTransport(m_shooter));
      }
}
