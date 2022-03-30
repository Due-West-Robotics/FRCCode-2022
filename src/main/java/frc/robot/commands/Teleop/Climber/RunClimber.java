// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop.Climber;

import frc.robot.subsystems.ClimberSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An intake command that uses the driveSubsystem. */
public class RunClimber extends CommandBase {
  private final ClimberSubsystem m_climberSubsystem;
  private DoubleSupplier m_leftSpeed, m_rightSpeed;
  private final double kSpeedMultiplier = -0.5;

  /**
   * Creates a new StartIntake command.
   *
   * @param intakeSubsystem The subsystem used by this command.
   */
  public RunClimber(ClimberSubsystem climberSubsystem, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
    m_climberSubsystem = climberSubsystem;
    m_leftSpeed = leftSpeed;
    m_rightSpeed = rightSpeed;
    addRequirements(m_climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climberSubsystem.setClimberSpeed(m_leftSpeed.getAsDouble() * kSpeedMultiplier, m_rightSpeed.getAsDouble() * kSpeedMultiplier);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.setClimberSpeed(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
