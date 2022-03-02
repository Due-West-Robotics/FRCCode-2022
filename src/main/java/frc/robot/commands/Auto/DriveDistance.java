package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistance extends CommandBase {
    private final DriveSubsystem m_drive;
    private boolean finished = false;
    //private double m_meters;
    //private double m_speed;
    //private double goal;
    
    public DriveDistance(DriveSubsystem driveSubsystem, double meters, double speed){
        m_drive = driveSubsystem;
        //m_meters = meters;
        //m_speed = speed;
        addRequirements(m_drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return finished;
    }
}
