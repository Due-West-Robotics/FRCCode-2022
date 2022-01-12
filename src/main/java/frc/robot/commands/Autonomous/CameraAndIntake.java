// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.commands.DrivingCommands.*;
import frc.robot.commands.CameraCommands.*;
import frc.robot.commands.IntakeCommands.*;

public class CameraAndIntake extends SequentialCommandGroup {

  DriveSubsystem m_drive;
  CameraSubsystem m_camera;
  IntakeSubsystem m_intake;

  /**
   * 
   * @param driveSubsystem The drive subsystem with which this sequential command will run
   * @param cameraSubsystem The camera subsystem with which this sequential command will run
   * @param intakeSubsystem The intake subsystem with which this sequential command will run
   * 
   */

  public CameraAndIntake(DriveSubsystem driveSubsystem, CameraSubsystem cameraSubsystem, IntakeSubsystem intakeSubsystem) {
    m_drive = driveSubsystem;
    m_camera = cameraSubsystem;
    m_intake = intakeSubsystem;

    addCommands(new CameraTurn(m_camera, m_drive),
                new DriveDistance(m_drive,-24, 0.65),
                new StartIntake(m_intake),
                new DriveDistance(m_drive,60, 0.35));

  }
}
