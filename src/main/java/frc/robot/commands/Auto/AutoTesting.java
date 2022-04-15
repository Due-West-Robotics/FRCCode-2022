package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoTesting extends SequentialCommandGroup{
    
    public AutoTesting(DriveSubsystem m_drive, ShooterSubsystem m_shooter, IntakeSubsystem m_intake) {
        addCommands(
            new AutoDriveTime(m_drive, 0.4, -0.4, 2.4, true));
      }
}
