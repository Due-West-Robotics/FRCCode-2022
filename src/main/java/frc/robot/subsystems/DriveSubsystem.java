// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  private CANSparkMax motor1L, motor2L, motor1R, motor2R;
  private RelativeEncoder encoderL, encoderR;
  private AHRS ahrs;
  private MotorControllerGroup leftMotors, rightMotors;
  private DifferentialDrive m_drive;
  private DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem.
   * @todo Fix error catching
   */
  public DriveSubsystem() {
    try {
      motor1L = new CANSparkMax(DriveConstants.kLeft1MotorPort,CANSparkMax.MotorType.kBrushless);
      motor2L = new CANSparkMax(DriveConstants.kLeft2MotorPort,CANSparkMax.MotorType.kBrushless);
      motor1R = new CANSparkMax(DriveConstants.kRight1MotorPort,CANSparkMax.MotorType.kBrushless);
      motor2R = new CANSparkMax(DriveConstants.kRight2MotorPort,CANSparkMax.MotorType.kBrushless);
      leftMotors = new MotorControllerGroup(motor1L, motor2L);
      rightMotors = new MotorControllerGroup(motor1R, motor2R);
      m_drive = new DifferentialDrive(leftMotors, rightMotors);
      encoderL = motor1L.getEncoder();
      encoderR = motor1R.getEncoder();
      resetEncoders();
      rightMotors.setInverted(true);
    }
    catch (Exception e){
      System.out.println("Motor setup error: " + e + "\n");
      e.printStackTrace();
    }
    try {
      ahrs = new AHRS(SPI.Port.kMXP);
      m_odometry = new DifferentialDriveOdometry(ahrs.getRotation2d());
    }
    catch (Exception e){
      System.out.println("Gyro error: " + e + "\n");
      e.printStackTrace();
    }
  }


  public void TankDrive(double left, double right){
    leftMotors.set(left * DriveConstants.kSpeedMultiplier);
    rightMotors.set(right * DriveConstants.kSpeedMultiplier);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  public void ArcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(encoderL.getVelocity(), encoderR.getVelocity());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, ahrs.getRotation2d());
  }

  public double getLeftEncoderPosition(){
    return encoderL.getPosition();
  }

  public double getRightEncoderPosition(){
    return encoderR.getPosition();
  }

  public void resetEncoders(){
    encoderL.setPosition(0.0);
    encoderR.setPosition(0.0);
  }

  public double getAverageEncoderDistance() {
    return (encoderL.getPosition() + encoderR.getPosition()) / 2.0;
  }

  public RelativeEncoder getLeftEncoder() {
    return encoderL;
  }

  public RelativeEncoder getRightEncoder() {
    return encoderR;
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    ahrs.zeroYaw();
  }

  public double getHeading() {
    return ahrs.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -ahrs.getRate();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(ahrs.getRotation2d(), encoderL.getPosition(), encoderR.getPosition());
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
