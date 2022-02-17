package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.Vision.CameraAim;

public class CancelCameraAim extends CommandBase{

  private final VisionSubsystem m_visionSubsystem;

  public CancelCameraAim(VisionSubsystem visionSubsystem) {
    m_visionSubsystem = visionSubsystem;
    addRequirements(m_visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!CommandScheduler.getInstance().isScheduled(new CameraAim(m_visionSubsystem, null, 0, 0))){
      
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}