// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax shooterMotor, transportMotor;
  private Servo shootingServoL, shootingServoR;


  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    try {
    shooterMotor = new CANSparkMax(ShooterConstants.kShooterMotorPort,CANSparkMax.MotorType.kBrushless);
    transportMotor = new CANSparkMax(ShooterConstants.kTransportMotorPort, CANSparkMax.MotorType.kBrushless);
    shootingServoL = new Servo(ShooterConstants.kShooterServoLPort);
    shootingServoR = new Servo(ShooterConstants.kShooterServoRPort);
    shootingServoL.setSpeed(ShooterConstants.kShooterServoSpeed);
    shootingServoR.setSpeed(ShooterConstants.kShooterServoSpeed);
    transportMotor.setIdleMode(IdleMode.kCoast);
    servoDown();
    }
    catch (Exception e){
      System.out.println("Shooter error: " + e + "\n");
      e.printStackTrace();
    }
  }

  public void servoDown(){
    shootingServoL.setAngle(20);
    shootingServoR.setAngle(160);
  }

  public void servoDownClose(){
    shootingServoL.setAngle(50);
    shootingServoR.setAngle(130);
  }

  public void servoUp(){
    shootingServoL.setAngle(90);
    shootingServoR.setAngle(90);
  }

  public void setShooterSpeed(double speed){
    shooterMotor.set(speed);
    System.out.println("setShooterSpeed called. Speed: " + speed);
  }

  public void setTransportSpeed(double speed){
    transportMotor.set(speed);
    System.out.println("setTransportSpeed called. Speed: " + speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Speed", shooterMotor.get());
    SmartDashboard.putNumber("Left Servo", shootingServoL.getAngle());
    SmartDashboard.putNumber("Right Servo", shootingServoR.getAngle());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
