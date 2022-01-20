package frc.robot.commands.IntakeCommands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
  
public class StopIntake extends CommandBase {
  
  private final IntakeSubsystem m_intake;

  /**
   * Creates a new StopIntake.
   *
   * @param intake The intake subsystem on which this command will run
   */
  public StopIntake(IntakeSubsystem intake) {
    m_intake = intake;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    //This stops the intake.
    //m_intake.stopIntake();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    //This makes the command exit instantly.
    return true;
  }
}
