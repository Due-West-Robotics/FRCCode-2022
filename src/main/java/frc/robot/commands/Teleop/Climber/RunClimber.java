// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop.Climber;

import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An intake command that uses the driveSubsystem. */
public class RunClimber extends CommandBase {
  private final ClimberSubsystem m_climberSubsystem;
  private boolean finished = false;

  /**
   * Creates a new StartIntake command.
   *
   * @param intakeSubsystem The subsystem used by this command.
   */
  public RunClimber(ClimberSubsystem climberSubsystem) {
    m_climberSubsystem = climberSubsystem;
    addRequirements(m_climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    m_climberSubsystem.setClimberSpeed(ClimberConstants.kClimberSpeed);
    //System.out.println("Starting Lift");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_climberSubsystem.getLeftClimberPosition() >= ClimberConstants.kClimberRaisedPosition){
      finished = true;
      //System.out.println("Running Lift");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.setClimberSpeed(0.0);
    //System.out.println("Stopped");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
