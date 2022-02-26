// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Teleop.*;
//import frc.robot.commands.Auto.*;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  private Joystick driveJoystick = new Joystick(0);

  private final Command m_TankDrive = new TankDrive(m_driveSubsystem, 
  () -> driveJoystick.getRawAxis(1),
  () -> driveJoystick.getRawAxis(5));
  
  private final Command m_startIntake = new StartIntake(m_intakeSubsystem);
  private final Command m_stopIntake = new StopIntake(m_intakeSubsystem);
  private final Command m_startShooter = new StartShooter(m_shooterSubsystem);
  private final Command m_stopShooter = new StopShooter(m_shooterSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_driveSubsystem.setDefaultCommand(new TankDrive(m_driveSubsystem,
    () -> driveJoystick.getRawAxis(1),
    () -> driveJoystick.getRawAxis(5)));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driveJoystick, OIConstants.kStartIntakeButton).whenPressed(new StartIntake(m_intakeSubsystem));
    new JoystickButton(driveJoystick, OIConstants.kStopIntakeButton).whenPressed(new StopIntake(m_intakeSubsystem));
    new JoystickButton(driveJoystick, OIConstants.kStartShooterButton).whenPressed(new StartShooter(m_shooterSubsystem));
    new JoystickButton(driveJoystick, OIConstants.kStopShooterButton).whenPressed(new StopShooter(m_shooterSubsystem));
    new JoystickButton(driveJoystick, OIConstants.kStartFeederButton).whenPressed(new FeedShooter(m_shooterSubsystem));
    new JoystickButton(driveJoystick, OIConstants.kStopFeederButton).whenPressed(new StopFeeder(m_shooterSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
