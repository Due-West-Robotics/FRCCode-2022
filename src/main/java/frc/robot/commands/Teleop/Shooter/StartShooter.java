// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop.Shooter;

import frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An intake command that uses the shooterSubsystem. */
public class StartShooter extends CommandBase {
  private final ShooterSubsystem m_shooterSubsystem;
  
  double m_power;
  /**
   * Creates a new StartShooter command.
   *
   * @param shooterSubsystem The subsystem used by this command.
   */
  public StartShooter(ShooterSubsystem shooterSubsystem, double power) {
    m_shooterSubsystem = shooterSubsystem;
    m_power = power;
    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSubsystem.setShooterSpeed(m_power);
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
