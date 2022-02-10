// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  private CANSparkMax motor1L;
  private CANSparkMax motor2L;
  private CANSparkMax motor1R;
  private CANSparkMax motor2R;
  private RelativeEncoder encoderL;
  private RelativeEncoder encoderR;
  private AHRS ahrs;


  /** Creates a new DriveSubsystem.
   * @todo Fix error catching
   */
  public DriveSubsystem() {
    try {
      motor1L = new CANSparkMax(DriveConstants.kLeft1MotorPort,CANSparkMax.MotorType.kBrushless);
      motor2L = new CANSparkMax(DriveConstants.kLeft2MotorPort,CANSparkMax.MotorType.kBrushless);
      motor1R = new CANSparkMax(DriveConstants.kRight1MotorPort,CANSparkMax.MotorType.kBrushless);
      motor2R = new CANSparkMax(DriveConstants.kRight2MotorPort,CANSparkMax.MotorType.kBrushless);
      encoderL = motor1L.getEncoder();
      encoderR = motor1R.getEncoder();
    }
    catch (Exception e){
      System.out.println("Motor error: " + e + "\n");
      e.printStackTrace();
    }

    try {
      ahrs = new AHRS(SPI.Port.kMXP);
    }
    catch (Exception e){
      System.out.println("Gyro error: " + e + "\n");
      e.printStackTrace();
    }
  }

  public void TankDrive(Double left, Double right){
    motor1L.set(-left * DriveConstants.kSpeedMultiplier);
    motor1R.set(-right * DriveConstants.kSpeedMultiplier);
  }

  public void ArcadeDrive(Double fwd, Double rot){
    double m_fwd = fwd;
    double m_rot = rot;

    double leftMotorOutput = 0;
    double rightMotorOutput = 0;

    leftMotorOutput += m_fwd;
    rightMotorOutput += m_fwd;

    leftMotorOutput += m_rot;
    rightMotorOutput += m_rot;

    leftMotorOutput = MathUtil.clamp(leftMotorOutput, -1.0, 1.0);
    rightMotorOutput = MathUtil.clamp(rightMotorOutput, -1.0, 1.0);

    motor1L.set(leftMotorOutput);
    motor1R.set(rightMotorOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Gyro", ahrs.getYaw());
    SmartDashboard.putNumber("Encoder L", encoderL.getPosition());
    SmartDashboard.putNumber("Encoder R", encoderR.getPosition());
    SmartDashboard.putNumber("Speed L", motor1L.get());
    SmartDashboard.putNumber("Speed R", motor1R.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
