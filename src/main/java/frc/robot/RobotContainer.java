// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

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
import frc.robot.commands.Auto.*;
import frc.robot.commands.Teleop.Climber.BrakeClimber;
import frc.robot.commands.Teleop.Climber.ClimberDoNothing;
import frc.robot.commands.Teleop.Climber.ReleaseClimber;
import frc.robot.commands.Teleop.Climber.RunClimber;
import frc.robot.commands.Teleop.Drive.*;
import frc.robot.commands.Teleop.Intake.*;
import frc.robot.commands.Teleop.Shooter.*;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
//import frc.robot.subsystems.VisionSubsystem;
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
  //private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();

  private Joystick leftDriveController = new Joystick(DriveConstants.kLeftControllerPort);
  private Joystick rightDriveController = new Joystick(DriveConstants.kRightControllerPort);
  private Joystick shootingController = new Joystick(DriveConstants.kShootingControllerPort);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

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

    // m_climberSubsystem.setDefaultCommand(new RunClimber(m_climberSubsystem,
    // () -> shootingController.getRawAxis(OIConstants.kLeftClimberAxis),
    // () -> shootingController.getRawAxis(OIConstants.kRightClimberAxis)));

    m_climberSubsystem.setDefaultCommand(new ClimberDoNothing(m_climberSubsystem));

    trajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(trajectory1JSON);
    trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(trajectory2JSON);

    m_chooser.setDefaultOption("Left", new AutoShootTwoBallLeft(m_driveSubsystem, m_shooterSubsystem, m_intakeSubsystem));
    m_chooser.addOption("Right", new AutoShootTwoBallRight(m_driveSubsystem, m_shooterSubsystem, m_intakeSubsystem));
    m_chooser.addOption("Middle", new AutoShootTwoBallMiddle(m_driveSubsystem, m_shooterSubsystem, m_intakeSubsystem));
    m_chooser.addOption("One Ball Auto", new AutoShoot(m_driveSubsystem, m_shooterSubsystem, m_intakeSubsystem));
    m_chooser.addOption("Two Ball Auto (Old)", new AutoShootWithExtraPickup(m_driveSubsystem, m_shooterSubsystem, m_intakeSubsystem));
    m_chooser.addOption("Nothing", new AutoDoNothing());
    m_chooser.addOption("Testing", new AutoTesting(m_driveSubsystem, m_shooterSubsystem, m_intakeSubsystem));

    try {
      m_chooser.addOption("Trajectory Path 1", new NavigateToPath(m_driveSubsystem, TrajectoryUtil.fromPathweaverJson(trajectoryPath1)));
      m_chooser.addOption("Trajectory Path 2", new NavigateToPath(m_driveSubsystem, TrajectoryUtil.fromPathweaverJson(trajectoryPath2)));
      m_chooser.addOption("AutoTest", new AutoTest(m_driveSubsystem, m_shooterSubsystem, m_intakeSubsystem));
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
    Button climberButton = new JoystickButton(shootingController, OIConstants.kRunClimberButton);
    Button transportButton = new JoystickButton(rightDriveController, OIConstants.kStartTransportButton);
    Button reverseIntake = new JoystickButton(shootingController, OIConstants.kReverseIntakeButton);
    Button startShooterLowGoalButton = new JoystickButton(shootingController, OIConstants.kStartShooterLowGoalButton);
    Button startShooterHighGoalCloseButton = new JoystickButton(shootingController, OIConstants.kStartShooterHighGoalCloseButton);
    Button startShooterHighGoalFarButton = new JoystickButton(shootingController, OIConstants.kStartShooterHighGoalFarButton);
    new POVButton(shootingController, 0).whenPressed(new LiftIntake(m_intakeSubsystem)); //dpad up
    new POVButton(shootingController, 180).whenPressed(new DropIntake(m_intakeSubsystem)); //dpad down
    climberButton.whenPressed(new RunClimber(m_climberSubsystem,
    () -> shootingController.getRawAxis(OIConstants.kRightClimberAxis),
    () -> shootingController.getRawAxis(OIConstants.kRightClimberAxis)));
    new JoystickButton(rightDriveController, OIConstants.kStartIntakeButton).whenPressed(new StartIntake(m_intakeSubsystem));
    new JoystickButton(leftDriveController, OIConstants.kStopIntakeButton).whenPressed(new StopIntake(m_intakeSubsystem));
    new JoystickButton(shootingController, OIConstants.kStopShooterButton).whenPressed(new StopShooter(m_shooterSubsystem));
    new JoystickButton(shootingController, OIConstants.kClimberBrakeButton).whenPressed(new BrakeClimber(m_climberSubsystem));
    new JoystickButton(shootingController, OIConstants.kClimberCoastButton).whenPressed(new ReleaseClimber(m_climberSubsystem));
    transportButton.whenPressed(new StartTransport(m_shooterSubsystem));
    transportButton.whenReleased(new StopTransport(m_shooterSubsystem));
    reverseIntake.whenPressed(new ParallelCommandGroup(new ReverseIntake(m_intakeSubsystem), new ReverseTransport(m_shooterSubsystem)));
    reverseIntake.whenReleased(new ParallelCommandGroup(new StopIntake(m_intakeSubsystem), new StopTransport(m_shooterSubsystem)));
    startShooterLowGoalButton.whenPressed(new SequentialCommandGroup(new StartShooter(m_shooterSubsystem, ShooterConstants.kShooterLowGoalSpeed), new ShooterHoodDown(m_shooterSubsystem, true)));
    startShooterHighGoalCloseButton.whenPressed(new SequentialCommandGroup(new StartShooter(m_shooterSubsystem, ShooterConstants.kShooterHighGoalCloseSpeed), new ShooterHoodDown(m_shooterSubsystem, false)));
    startShooterHighGoalFarButton.whenPressed(new SequentialCommandGroup(new StartShooter(m_shooterSubsystem, ShooterConstants.kShooterHighGoalFarSpeed), new ShooterHoodUp(m_shooterSubsystem)));
  }

  public void setDrivetrainCoast(){
    m_driveSubsystem.setCoast();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return new NavigateToPath(m_driveSubsystem);
    return m_chooser.getSelected();
  }
}
