// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  private CANSparkMax climbMotor1, climbMotor2;
  private RelativeEncoder climbMotor1Encoder, climbMotor2Encoder;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    try{
      climbMotor1 = new CANSparkMax(ClimberConstants.kClimber1MotorPort,CANSparkMax.MotorType.kBrushless);
      climbMotor2 = new CANSparkMax(ClimberConstants.kClimber2MotorPort,CANSparkMax.MotorType.kBrushless);
      climbMotor1Encoder = climbMotor1.getEncoder();
      climbMotor2Encoder = climbMotor2.getEncoder();
      climbMotor2.setInverted(true);

      setClimberBrake();
    }
    catch(Exception e){
      System.out.println("Climber error: " + e + "\n");
      e.printStackTrace();
    }
  }

  public void setClimberSpeed(double leftSpeed, double rightSpeed){
    climbMotor1.set(rightSpeed);
    climbMotor2.set(leftSpeed);
    System.out.println("setClimberSpeed called. Speed: " + leftSpeed);
  }
  
  public double getLeftClimberPosition() {
    return climbMotor2Encoder.getPosition();
  }

  public double getRightClimberPosition() {
    return climbMotor1Encoder.getPosition();
  }

  public void setClimberBrake() {
    climbMotor1.setIdleMode(IdleMode.kBrake);
    climbMotor2.setIdleMode(IdleMode.kBrake);
  }

  public void setClimberCoast() {
    climbMotor1.setIdleMode(IdleMode.kCoast);
    climbMotor2.setIdleMode(IdleMode.kCoast);
  }

  public boolean climberBrakeOn() {
    if (climbMotor1.getIdleMode() == IdleMode.kBrake){
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Climber", getLeftClimberPosition());
    SmartDashboard.putNumber("Right Climber", getRightClimberPosition());
    SmartDashboard.putBoolean("Climber Brake", climberBrakeOn());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
