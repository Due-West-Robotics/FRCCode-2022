package frc.robot.commands.DrivingCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes - actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.RunCommand}.
 */
public class SlowJoystickDrive extends CommandBase {
  private final DriveSubsystem m_drive;
  private final DoubleSupplier m_forward;
  private final DoubleSupplier m_rotation;
  private double m_fwd, m_rot;

  /**
   * Creates a new SlowJoyStickDrive
   *
   * @param subsystem The drive subsystem this command wil run on.
   * @param forward  The control input for driving forwards/backwards
   * @param rotation The control input for turning
   */
  public SlowJoystickDrive(DriveSubsystem subsystem, DoubleSupplier forward, DoubleSupplier rotation) {
    m_drive = subsystem;
    m_forward = forward;
    m_rotation = rotation;
    addRequirements(m_drive);
  }

  @Override
  public void execute() {

     // @todo Add max speed of 70%
     
    //get the sign of each value (needed for squared sensitivity)
    double fwdSign = Math.signum(-m_forward.getAsDouble());
    double rotSign = Math.signum(m_rotation.getAsDouble());

    m_fwd = Math.pow(m_forward.getAsDouble(),2) * DriveConstants.kSlow;
    m_rot = Math.pow(m_rotation.getAsDouble(),4) * DriveConstants.kSlow;
  

    //add thresholds for very low power
    if(Math.abs(m_fwd) < DriveConstants.kMinPower) {
      m_fwd = 0;
    }
    if(Math.abs(m_rot) < DriveConstants.kMinPower) {
      m_rot = 0;
    }
    if(fwdSign < 0) {
      m_fwd *= -1;
    }
    if(rotSign < 0) {
      m_rot *= -1;
    }
    
    if(Math.abs(m_fwd) + Math.abs(m_rot) < DriveConstants.kMinPower) {
      m_drive.resetIAccum();
    }
    m_drive.arcadeDrive(m_fwd, m_rot);

    SmartDashboard.putNumber("Target Speed", m_fwd);
  }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

      //Turn off integral control, which causes problems with quickly changing values
      //m_drive.setPID_I(0);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_drive.arcadeDrive(0, 0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}