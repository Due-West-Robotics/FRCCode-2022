// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants.*;
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
      encoderL.setVelocityConversionFactor(0.00398982267005903741284755709676/10.71);
      encoderR.setVelocityConversionFactor(0.00398982267005903741284755709676/10.71);
      encoderL.setPositionConversionFactor(0.00398982267005903741284755709676/10.71);
      encoderR.setPositionConversionFactor(0.00398982267005903741284755709676/10.71);
      leftMotors.setInverted(true);
      resetEncoders();
    }
    catch (Exception e){
      System.out.println("Motor setup error: " + e + "\n");
      e.printStackTrace();
    }
    try {
      ahrs = new AHRS(SPI.Port.kMXP);
      //m_odometry = new DifferentialDriveOdometry(ahrs.getRotation2d());
    }
    catch (Exception e){
      System.out.println("Gyro error: " + e + "\n");
      e.printStackTrace();
    }
  }

  public void TankDrive(double left, double right){
    m_drive.tankDrive(left * DriveConstants.kSpeedMultiplier, right * DriveConstants.kSpeedMultiplier);
    m_drive.feed();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  public void ArcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
    m_drive.feed();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(encoderL.getVelocity(), encoderR.getVelocity());
  }

  // public void resetOdometry(Pose2d pose) {
  //   resetEncoders();
  //   m_odometry.resetPosition(pose, ahrs.getRotation2d());
  // }

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

  public void setBrake() {
    motor1L.setIdleMode(IdleMode.kBrake);
    motor2L.setIdleMode(IdleMode.kBrake);
    motor1R.setIdleMode(IdleMode.kBrake);
    motor2R.setIdleMode(IdleMode.kBrake);
  }

  public void setCoast() {
    motor1L.setIdleMode(IdleMode.kCoast);
    motor2L.setIdleMode(IdleMode.kCoast);
    motor1R.setIdleMode(IdleMode.kCoast);
    motor2R.setIdleMode(IdleMode.kCoast);
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public void calibrateGyro() {
    ahrs.calibrate();
  }

  public boolean gyroIsCalibrating() {
    return ahrs.isCalibrating();
  }

  public void zeroHeading() {
    ahrs.reset();
  }

  public double getHeading() {
    return ahrs.getRotation2d().getDegrees();
  }

  public double getRotation() {
    return ahrs.getRoll();
  }

  public double getTurnRate() {
    return -ahrs.getRate();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(ahrs.getRotation2d(), encoderL.getPosition(), encoderR.getPosition());
    SmartDashboard.putNumber("Roll", ahrs.getRoll());
    SmartDashboard.putNumber("Pitch", ahrs.getPitch());
    SmartDashboard.putNumber("Yaw", getPose().getRotation().getRadians());
    SmartDashboard.putNumber("Encoder L", encoderL.getPosition());
    SmartDashboard.putNumber("Encoder R", encoderR.getPosition());
    SmartDashboard.putNumber("Speed L", leftMotors.get());
    SmartDashboard.putNumber("Speed R", rightMotors.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
