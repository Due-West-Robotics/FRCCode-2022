package frc.robot.commands.Teleop.Climber;

import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class StartClimber extends CommandBase{

  private final ClimberSubsystem m_climberSubsystem;

  public StartClimber(ClimberSubsystem climberSubsystem) {
    m_climberSubsystem = climberSubsystem;
    addRequirements(m_climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climberSubsystem.setClimberSpeed(ClimberConstants.kClimberSpeed);
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