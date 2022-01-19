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
public class FastTankJoystickDrive extends CommandBase {
  private final DriveSubsystem m_drive;
  private final DoubleSupplier m_left;
  private final DoubleSupplier m_right;
  private double m_l, m_r;

  /**
   * Creates a new FastJoyStickDrive
   *
   * @param subsystem The drive subsystem this command wil run on.
   * @param forward  The control input for driving forwards/backwards
   * @param rotation The control input for turning
   */
  public FastTankJoystickDrive(DriveSubsystem subsystem, DoubleSupplier left, DoubleSupplier right) {
    m_drive = subsystem;
    m_left = left;
    m_right = right;
    addRequirements(m_drive);
  }

  @Override
  public void execute() {

     // @todo Add max speed of 70%
     
    //get the sign of each value (needed for squared sensitivity)
    double leftSign = Math.signum(-m_left.getAsDouble());
    double rightSign = Math.signum(-m_right.getAsDouble());

    m_l = Math.pow(m_left.getAsDouble(),2) * DriveConstants.kFast;
    m_r = Math.pow(m_right.getAsDouble(),2) * DriveConstants.kFast;
  

    //add thresholds for very low power
    if(Math.abs(m_l) < DriveConstants.kMinPower) {
      m_l = 0;
    }
    if(Math.abs(m_r) < DriveConstants.kMinPower) {
      m_r = 0;
    }
    if(leftSign < 0) {
      m_l *= -1;
    }
    if(rightSign < 0) {
      m_r *= -1;
    }
    
    if(Math.abs(m_l) + Math.abs(m_r) < DriveConstants.kMinPower) {
      m_drive.resetIAccum();
    }
    m_drive.tankDrive(m_l, m_r);

    SmartDashboard.putBoolean("Fast Mode", true);
    SmartDashboard.putNumber("Target Left Speed", m_l);
    SmartDashboard.putNumber("Target Right Speed", m_r);

    
  }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

      SmartDashboard.putBoolean("Fast Mode", true);

      //Turn off integral control, which causes problems with quickly changing values
      //m_drive.setPID_I(0);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_drive.tankDrive(0, 0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}