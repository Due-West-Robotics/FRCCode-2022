package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Teleop.Intake.DropIntake;
import frc.robot.commands.Teleop.Intake.StartTransport;
import frc.robot.commands.Teleop.Intake.StopTransport;
import frc.robot.commands.Teleop.Shooter.ShooterHoodDown;
import frc.robot.commands.Teleop.Shooter.StartShooter;
import frc.robot.commands.Teleop.Shooter.StopShooter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShoot extends SequentialCommandGroup{
    
    public AutoShoot(DriveSubsystem m_drive, ShooterSubsystem m_shooter, IntakeSubsystem m_intake) {
        addCommands(
            new ShooterHoodDown(m_shooter),
            new StartShooter(m_shooter, ShooterConstants.kShooterHighGoalCloseSpeed),
            new WaitCommand(3),
            new StartTransport(m_shooter),
            new WaitCommand(3),
            new StopShooter(m_shooter),
            new StopTransport(m_shooter));
      }


}
