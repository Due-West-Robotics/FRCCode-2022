// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  private final CANSparkMax motor1L = new CANSparkMax(DriveConstants.kLeft1MotorPort,CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax motor2L = new CANSparkMax(DriveConstants.kLeft2MotorPort,CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax motor1R = new CANSparkMax(DriveConstants.kRight1MotorPort,CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax motor2R = new CANSparkMax(DriveConstants.kRight2MotorPort,CANSparkMax.MotorType.kBrushless);
  private final RelativeEncoder encoderL = motor1L.getEncoder();
  private final RelativeEncoder encoderR = motor1R.getEncoder();
  private final AHRS ahrs = new AHRS(SPI.Port.kMXP);


  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    motor1R.setInverted(true);
    motor2R.setInverted(true);
    motor2L.follow(motor1L);
    motor2R.follow(motor1R);
  }

  public void TankDrive(Double left, Double right){
    motor1L.set(left);
    motor1R.set(right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Gyro", ahrs.getYaw());
    SmartDashboard.putNumber("Encoder L", encoderL.getPosition());
    SmartDashboard.putNumber("Encoder R", encoderR.getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
