package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TestMeterDrive extends CommandBase {
    private final DriveSubsystem m_drive;
    private boolean finished = false;
    
    public TestMeterDrive(DriveSubsystem driveSubsystem){
        m_drive = driveSubsystem;
        addRequirements(m_drive);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_drive.TankDrive(0.05, 0.05);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_drive.getLeftEncoderPosition() > 1){
            finished = true;
        }
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
