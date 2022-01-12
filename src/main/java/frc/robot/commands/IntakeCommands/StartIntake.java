package frc.robot.commands.IntakeCommands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
  
public class StartIntake extends CommandBase {
  
  private final IntakeSubsystem m_intake;

  /**
   * Creates a new StartIntake.
   *
   * @param intake The intake subsystem on which this command will run
   */
  public StartIntake(IntakeSubsystem intake) {
    m_intake = intake;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    //This starts the intake.
    m_intake.startIntake();
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
