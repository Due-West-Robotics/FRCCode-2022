// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AutoDriveTime extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private double m_left, m_right, m_time;
  private boolean finished, m_brake = false;
  private Timer timer;
  /**
   * Creates a new TankDrive command.
   *
   * @param driveSubsystem The subsystem used by this command.
   * @param left Left motor speed
   * @param right right motor speed
   * @param time
   * @param brake
   */
  public AutoDriveTime(DriveSubsystem driveSubsystem, double left, double right, double time, boolean brake) {
    m_driveSubsystem = driveSubsystem;
    timer = new Timer();
    m_left = left;
    m_right = right;
    m_time = time;
    m_brake = brake;
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_brake){
      m_driveSubsystem.setBrake();
    }
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.TankDrive(m_left, m_right);
    if (timer.get() >= m_time){
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.TankDrive(0.0, 0.0);
    m_driveSubsystem.setCoast();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
