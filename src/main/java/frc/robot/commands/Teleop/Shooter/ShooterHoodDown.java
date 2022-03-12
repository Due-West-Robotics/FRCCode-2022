// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop.Shooter;

import frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An intake command that uses the driveSubsystem. */
public class ShooterHoodDown extends CommandBase {
  private final ShooterSubsystem m_shooterSubsystem;
  private boolean m_close;

  /**
   * Creates a new StartShooter command.
   *
   * @param shooterSubsystem The subsystem used by this command.
   * @param close
   */
  public ShooterHoodDown(ShooterSubsystem shooterSubsystem, boolean close) {
    m_shooterSubsystem = shooterSubsystem;
    m_close = close;
    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_close){
      m_shooterSubsystem.servoDownClose();
    }
    else {
      m_shooterSubsystem.servoDown();
    }
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
