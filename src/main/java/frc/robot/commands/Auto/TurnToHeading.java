package frc.robot.commands.Auto;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnToHeading extends CommandBase {
    
    private final DriveSubsystem m_driveSubsystem;
    private double m_power;
    private Rotation2d m_targetHeading;
    private boolean finished = false;
    private double turnTolerance = Math.PI / 16;

    /**
     * Constructor
     *
     * @param driveSubsystem The subsystem used by this command.
     * @param targetHeading The ending heading.
     * @param power The power, between 0 and 1.
     */
    public TurnToHeading(DriveSubsystem driveSubsystem, Rotation2d targetHeading, double power) {
      m_driveSubsystem = driveSubsystem;
      m_targetHeading = targetHeading;
      m_power = power;
      addRequirements(m_driveSubsystem);
    }

    /**
     * Constructor
     *
     * @param driveSubsystem The subsystem used by this command.
     * @param targetHeading The ending heading, in rad.
     * @param power The power, between 0 and 1.
     */
    public TurnToHeading(DriveSubsystem driveSubsystem, double targetHeading, double power) {
        m_driveSubsystem = driveSubsystem;
        m_targetHeading = new Rotation2d(targetHeading);
        m_power = power;
        addRequirements(m_driveSubsystem);
      }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(m_targetHeading.getRadians() - turnTolerance > m_driveSubsystem.getPose().getRotation().getRadians()) {
            m_driveSubsystem.TankDrive(m_power, -m_power);
        } else if(m_targetHeading.getRadians() + turnTolerance < m_driveSubsystem.getPose().getRotation().getRadians()) {
            m_driveSubsystem.TankDrive(-m_power, m_power);
        } else {
            finished = true;
        }
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_driveSubsystem.TankDrive(0.0, 0.0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return finished;
    }
  
}
