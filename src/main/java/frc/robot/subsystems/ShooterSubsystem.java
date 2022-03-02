// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax shooterMotor;
  private Servo shootingServoL, shootingServoR;


  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    try {
    shooterMotor = new CANSparkMax(ShooterConstants.kShooterMotorPort,CANSparkMax.MotorType.kBrushless);
    shootingServoL = new Servo(ShooterConstants.kShooterServoLPort);
    shootingServoR = new Servo(ShooterConstants.kShooterServoRPort);
    shootingServoL.setSpeed(ShooterConstants.kShooterServoSpeed);
    shootingServoR.setSpeed(ShooterConstants.kShooterServoSpeed);
    }
    catch (Exception e){
      System.out.println("Shooter error: " + e + "\n");
      e.printStackTrace();
    }
  }

  public void servoDown(){
    shootingServoL.setAngle(0);
    shootingServoR.setAngle(0);
  }

  public void servoUp(){
    shootingServoL.setAngle(180);
    shootingServoR.setAngle(180);
  }

  public void setShooterSpeed(double speed){
    shooterMotor.set(speed);
    System.out.println("setShooterSpeed called. Speed: " + speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Speed", shooterMotor.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
