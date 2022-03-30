package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnDegrees extends CommandBase {
    private final DriveSubsystem m_drive;
    private boolean finished = false;
    private double m_degrees, m_speed;
    private int m_direction;
    
    /**
     * 
     * @param driveSubsystem The drivesubsystem used for this command.
     * @param degrees The goal degrees that is absolute. Gyro is -180 to 180
     * @param direction The direction of the turn. 0 = left, 1 = right
     * @param speed The speed of the turn.
     */
    public TurnDegrees(DriveSubsystem driveSubsystem, double degrees, int direction, double speed){
        m_drive = driveSubsystem;
        m_degrees = degrees;
        m_direction = direction;
        m_speed = speed;
        addRequirements(m_drive);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drive.TankDrive(0.0, 0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return finished;
    }

}
