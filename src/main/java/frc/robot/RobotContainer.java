// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Auto.AutoShoot;
import frc.robot.commands.Teleop.Drive.*;
import frc.robot.commands.Teleop.Intake.*;
import frc.robot.commands.Teleop.Shooter.*;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

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
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();

  private Joystick leftDriveController = new Joystick(DriveConstants.kLeftControllerPort);
  private Joystick rightDriveController = new Joystick(DriveConstants.kRightControllerPort);
  private Joystick shootingController = new Joystick(DriveConstants.kShootingControllerPort);

  SendableChooser<Trajectory> m_chooser = new SendableChooser<>();

  String trajectory1JSON = "paths/YourPath.wpilib.json";
  String trajectory2JSON = "paths/YourPath2.wpilib.json";
  Path trajectoryPath1, trajectoryPath2;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_driveSubsystem.setDefaultCommand(new TankDrive(m_driveSubsystem,
    () -> leftDriveController.getRawAxis(1),
    () -> rightDriveController.getRawAxis(1)));

    trajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(trajectory1JSON);
    trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(trajectory2JSON);

    try {
      m_chooser.setDefaultOption("Path 1", TrajectoryUtil.fromPathweaverJson(trajectoryPath1));
    } catch (IOException e) {
      e.printStackTrace();
    }
    try {
      m_chooser.addOption("Path 2", TrajectoryUtil.fromPathweaverJson(trajectoryPath2));
    } catch (IOException e) {
      e.printStackTrace();
    }
    Shuffleboard.getTab("Autonomous").add(m_chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new POVButton(shootingController, 0).whenPressed(new LiftIntake(m_intakeSubsystem)); //dpad up
    new POVButton(shootingController, 180).whenPressed(new DropIntake(m_intakeSubsystem)); //dpad down
    new JoystickButton(rightDriveController, OIConstants.kStartIntakeButton).whenPressed(new StartIntake(m_intakeSubsystem));
    new JoystickButton(leftDriveController, OIConstants.kStopIntakeButton).whenPressed(new StopIntake(m_intakeSubsystem));
    new JoystickButton(shootingController, OIConstants.kStopShooterButton).whenPressed(new StopShooter(m_shooterSubsystem));
    Button transportButton = new JoystickButton(rightDriveController, OIConstants.kStartTransportButton);
    transportButton.whenPressed(new StartTransport(m_shooterSubsystem));
    transportButton.whenReleased(new StopTransport(m_shooterSubsystem));
    Button reverseIntake = new JoystickButton(shootingController, OIConstants.kReverseIntakeButton);
    reverseIntake.whenPressed(new ParallelCommandGroup(new ReverseIntake(m_intakeSubsystem), new ReverseTransport(m_shooterSubsystem)));
    reverseIntake.whenReleased(new ParallelCommandGroup(new StopIntake(m_intakeSubsystem), new StopTransport(m_shooterSubsystem)));
    Button startShooterLowGoalButton = new JoystickButton(shootingController, OIConstants.kStartShooterLowGoalButton);
    startShooterLowGoalButton.whenPressed(new SequentialCommandGroup(new StartShooter(m_shooterSubsystem, ShooterConstants.kShooterLowGoalSpeed), new ShooterHoodDown(m_shooterSubsystem, true)));
    Button startShooterHighGoalCloseButton = new JoystickButton(shootingController, OIConstants.kStartShooterHighGoalCloseButton);
    startShooterHighGoalCloseButton.whenPressed(new SequentialCommandGroup(new StartShooter(m_shooterSubsystem, ShooterConstants.kShooterHighGoalCloseSpeed), new ShooterHoodDown(m_shooterSubsystem, false)));
    Button startShooterHighGoalFarButton = new JoystickButton(shootingController, OIConstants.kStartShooterHighGoalFarButton);
    startShooterHighGoalFarButton.whenPressed(new SequentialCommandGroup(new StartShooter(m_shooterSubsystem, ShooterConstants.kShooterHighGoalFarSpeed), new ShooterHoodUp(m_shooterSubsystem)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return new NavigateToPath(m_driveSubsystem);
    return new AutoShoot(m_driveSubsystem, m_shooterSubsystem, m_intakeSubsystem);
  }
}
