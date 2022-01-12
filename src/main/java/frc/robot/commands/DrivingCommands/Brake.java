package frc.robot.commands.DrivingCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

/**
 * A command to stop the robot.
 */
public class Brake extends CommandBase {
  private final DriveSubsystem m_drive;

  /**
   * Creates a new Brake.
   *
   * @param subsystem The drive subsystem this command wil run on.
   */
  public Brake(DriveSubsystem subsystem) {
      m_drive = subsystem;
      addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      m_drive.setBrake();
    }

    @Override
    public void execute() {
    }

  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return true;
    }
      
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}