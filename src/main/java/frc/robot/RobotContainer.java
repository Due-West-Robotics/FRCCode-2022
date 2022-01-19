// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.*;
//import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DrivingCommands.*;
import frc.robot.commands.IntakeCommands.*;
import frc.robot.commands.Autonomous.*;
import frc.robot.subsystems.IntakeSubsystem;

/** We removed any reference to the Camera Subsystem from this file since we will not be
 *  using the camera in the competition.
*/




/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  //private final CameraSubsystem m_cameraSubsystem = new CameraSubsystem();

  // The autonomous routines

  //A chooser for autonomous commands
  //SendableChooser<Command> m_chooser = new SendableChooser<>();


  // The driver's controller
  GenericHID m_driverController = new Joystick(0);

  private final Command m_arcadeDrive = new SlowArcadeJoystickDrive(m_robotDrive,
  () -> m_driverController.getRawAxis(1),
  () -> m_driverController.getRawAxis(0));

  private final Command m_autoCommand = new CompetitionAuto(m_robotDrive, m_intakeSubsystem);

  //private final Command m_defaultDrive = new DefaultDrive(m_robotDrive);

  /*private final Command myBarrelRacing = new BarrelRacing(m_robotDrive);
  private final Command myBouncePath = new BouncePath(m_robotDrive);
  private final Command mySlalomPath = new SlalomPath(m_robotDrive);
  private final Command myTestCommand = new AutoTest(m_robotDrive);

  private final Command pathABlue = new PathABlue(m_robotDrive, m_intakeSubsystem);
  private final Command pathARed = new PathARed(m_robotDrive, m_intakeSubsystem);
  private final Command pathBBlue = new PathBBlue(m_robotDrive, m_intakeSubsystem);
  private final Command pathBRed = new PathBRed(m_robotDrive, m_intakeSubsystem);

  private Command GalacticChooser() {
    System.out.println("Galactic Chooser called.");
    if (m_cameraSubsystem.GetTargetHorizontalOffset() > -3 && m_cameraSubsystem.GetTargetHorizontalOffset() < 3 && m_cameraSubsystem.GetTargetArea() > 0.7){
      System.out.println("Path A Red");
      return pathARed;
    }
    else if (m_cameraSubsystem.GetTargetHorizontalOffset() > 20){
      System.out.println("Path B Red");
      return pathBRed;
    }
    else if (m_cameraSubsystem.GetTargetArea() < 2 && Math.abs(m_cameraSubsystem.GetTargetHorizontalOffset()) < 5) {
      System.out.println("Path A Blue");
      return pathABlue;
    }
    else {
      return pathBBlue;
    }
  }*/

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(


        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        // new SlowArcadeJoystickDrive(
        //     m_robotDrive,
        //     () -> m_driverController.getRawAxis(1),
        //     () -> m_driverController.getRawAxis(0)));


        new SlowTankJoystickDrive(
            m_robotDrive,
            () -> m_driverController.getRawAxis(1),
            () -> m_driverController.getRawAxis(5)));


    /*m_intakeSubsystem.setDefaultCommand(
      // A split-stick arcade command, with forward/backward controlled by the left
      // hand, and turning controlled by the right.

      new DefaultIntake(
          m_intakeSubsystem));
      

   // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Barrel Racing", myBarrelRacing);
    m_chooser.addOption("Bounce Path", myBouncePath);
    m_chooser.addOption("Slalom Path", mySlalomPath);
    m_chooser.addOption("AutoTest", myTestCommand);
    m_chooser.addOption("Galactic Search", GalacticChooser());

    //Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);
    */
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, OIConstants.kButtonIntakeOn).whenPressed(new StartIntake(m_intakeSubsystem));
    new JoystickButton(m_driverController, OIConstants.kButtonIntakeOff).whenPressed(new StopIntake(m_intakeSubsystem));
    //new JoystickButton(m_driverController, OIConstants.kButtonIntakeReverse).whenPressed(new ReverseIntake(m_intakeSubsystem));
    //new JoystickButton(m_driverController, OIConstants.kButtonLaunchIntake).whenPressed(new LaunchIntake(m_intakeSubsystem));
    new JoystickButton(m_driverController, OIConstants.kButtonFastGear).whenPressed(new FastTankJoystickDrive(m_robotDrive,
    () -> m_driverController.getRawAxis(1),
    () -> m_driverController.getRawAxis(5)));
    new JoystickButton(m_driverController, OIConstants.kButtonSlowGear).whenPressed(new SlowTankJoystickDrive(m_robotDrive,
    () -> m_driverController.getRawAxis(1),
    () -> m_driverController.getRawAxis(5)));
    }
  
  public Command getAutonomousCommand() {
    return m_autoCommand;
  }

  public void arcadeDrive() {
    m_arcadeDrive.schedule();
  }

  public void resetGyro(){
    m_robotDrive.resetGyro();
  }

  public double getGyro(){
    return m_robotDrive.getGyro();
  }

  public void calibrateGyro() {
    m_robotDrive.calibrateGyro();
  }

  public void encoderInit(){
    m_robotDrive.setDistancePerPulse();
  }

  public void resetEncoders() {
    m_robotDrive.resetEncoders();
  }
  public Command testCommand() {
    Command testCommand = new AutoTest(m_robotDrive);
    return testCommand;
  }
}