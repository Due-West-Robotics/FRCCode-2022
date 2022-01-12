// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CameraCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CameraSubsystem;

public class DefaultCamera extends CommandBase {
  private final CameraSubsystem m_camera;
  /**
   * Creates a new DefaultCamera.
   *
   * @param m_cameraSubsystem The camera subsystem this command wil run on.
   */
  public DefaultCamera(CameraSubsystem m_cameraSubsystem) {
    m_camera = m_cameraSubsystem;
    addRequirements(m_camera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
