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

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax intakeMotor, intakeLifterMotor;
  private RelativeEncoder intakeLifterMotorEncoder;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    try {
      intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorPort,CANSparkMax.MotorType.kBrushless);
      intakeLifterMotor = new CANSparkMax(IntakeConstants.kIntakeLiftMotorPort, CANSparkMax.MotorType.kBrushless);
      intakeLifterMotorEncoder = intakeLifterMotor.getEncoder();
      intakeLifterMotorEncoder.setPosition(0.0);
      intakeLifterMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
      intakeLifterMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
      intakeLifterMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 40.0f);
      intakeLifterMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -40.0f);
    }
    catch (Exception e){
      System.out.println("Intake error: " + e + "\n");
      e.printStackTrace();
    }
  }

  public void setIntakeSpeed(double speed){
    intakeMotor.set(speed);
    System.out.println("setIntakeSpeed called. Speed: " + speed);
  }

  public void setIntakeLifterSpeed(double speed){
    System.out.println("setIntakeLifterSpeed called. setLifted = " + speed);
    intakeLifterMotor.set(speed);
  }

  public void stopIntakeLifter(){
    System.out.println("stopIntakeLifter called.");
    intakeLifterMotor.set(0.0f);
    intakeMotor.stopMotor();
  }

  public void setLifterBrake(){
    intakeLifterMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setLifterCoast(){
    intakeLifterMotor.setIdleMode(IdleMode.kCoast);
  }

  public double getIntakeLifterMotorPosition(){
    return intakeLifterMotorEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Speed", intakeMotor.get());
    SmartDashboard.putNumber("Intake Lifter Position", getIntakeLifterMotorPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
