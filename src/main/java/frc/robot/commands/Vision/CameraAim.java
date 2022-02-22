package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;

public class CameraAim extends CommandBase {
    private final VisionSubsystem m_camera;
    private final DriveSubsystem m_drive;
    private int m_target, m_direction;
    private boolean finished = false;
    /**
     * Creates a new CameraAim.
     * This is a command that turns the robot until it finds a target.
     *
     * @param m_cameraSubsystem The camera subsystem this command will run on.
     * @param m_driveSubsystem The drive subsystem this command will run on.
     * @param target The target of the command. 0 = Red, 1 = Blue, 2 = Target.
     * @param direction The direction for the search. 0 = left, 1 = right.
     */
    public CameraAim(VisionSubsystem CameraSubsystem, DriveSubsystem DriveSubsystem, int target, int direction) {
      m_camera = CameraSubsystem;
      m_drive = DriveSubsystem;
      m_target = target;
      m_direction = direction;
      addRequirements(m_drive);
  }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      if (m_target == VisionConstants.kRedPipeline || m_target == VisionConstants.kBluePipeline || m_target == VisionConstants.kTargetPipeline){
        m_camera.SetActivePipeline(m_target);
      }
      else {
        System.out.println("CameraAim target invalid: " + m_target);
        cancel();
      }
      if (m_direction != 0 && m_direction != 1){
        System.out.println("CameraAim direction invalid: " + m_direction);
        cancel();
      }
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if (m_camera.HasValidTarget()) {
          if (m_camera.GetTargetHorizontalOffset() > VisionConstants.kCameraHorizontalGoal) {
            m_drive.ArcadeDrive(0.5, 0.25);
          }
          else if (m_camera.GetTargetHorizontalOffset() < (VisionConstants.kCameraHorizontalGoal * -1)){
            m_drive.ArcadeDrive(0.5, -0.25);
          }
          else {
            if (m_camera.GetTargetArea() > VisionConstants.kCameraTargetAreaGoal || m_camera.GetTargetVerticalOffset() > VisionConstants.kCameraVerticalGoal){
              m_drive.ArcadeDrive(-0.25, 0.0);
            }
            else if (m_camera.GetTargetArea() < VisionConstants.kCameraTargetAreaGoal || m_camera.GetTargetVerticalOffset() < VisionConstants.kCameraVerticalGoal) {
              m_drive.ArcadeDrive(0.25, 0.0);
            }
            else {
              finished = true;
            }
          }
        }
        else {
          if (m_direction == 0){
            m_drive.ArcadeDrive(0.0, (VisionConstants.kTurnSpeed * -1));
          }
          else {
            m_drive.ArcadeDrive(0.0, VisionConstants.kTurnSpeed);
          }
        }

      }

  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_drive.ArcadeDrive(0.0, 0.0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return finished;
    }
}