// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.AtHomePaths.AutoNav;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.*;
import frc.robot.commands.DrivingCommands.*;

public class BarrelRacing extends SequentialCommandGroup {

  DriveSubsystem m_drive;

   /**
   * Creates a new autonomous BarrelRacing command.
   *
   * @param driveSubsystem The drive subsystem with which this sequential command will run
   */

  public BarrelRacing(DriveSubsystem driveSubsystem) {
    m_drive = driveSubsystem;
    addCommands(
    new DriveDistance(m_drive, 112, 1),
    new TurnDegrees(m_drive, 360, 0.2,DriveConstants.kRight,30),
    new DriveDistance(m_drive, 100, 1),
    new TurnDegrees(m_drive, 45, 0.15,DriveConstants.kLeft,30),
    new DriveDistance(m_drive, 86, 1),
    new TurnDegrees(m_drive, -178, 0.15,DriveConstants.kLeft,25.811),
    new DriveDistance(m_drive, 255, 1));
    }
}
