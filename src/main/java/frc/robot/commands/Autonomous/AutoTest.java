package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.DrivingCommands.*;

public class AutoTest extends SequentialCommandGroup {

    DriveSubsystem m_drive;

    /**
     * Creates a new autonomous AutoTest command.
     * This is our robot testing program.
     * 
     * @param driveSubsystem the drive subsystem with which this sequential command will run
     * 
     */

    public AutoTest(DriveSubsystem driveSubsystem) {
        m_drive = driveSubsystem;
        addCommands(
            new DriveDistance(m_drive, 24, .2, .2),
            new DriveDistance(m_drive, 24, .3),
            new DriveDistance(m_drive, -48, .3));
    }
}