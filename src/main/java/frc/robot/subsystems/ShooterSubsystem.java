// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final CANSparkMax shooterMotor = new CANSparkMax(ShooterConstants.kShooterMotorPort,CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax shooterFeedMotor = new CANSparkMax(ShooterConstants.kShooterFeedPort,CANSparkMax.MotorType.kBrushless);
  private final Double servoSpeed = 0.5;
  private final Servo intakeServo = new Servo(IntakeConstants.kIntakeServoPort);


  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    intakeServo.setSpeed(servoSpeed);
  }

  public void servoDown() {
    intakeServo.setAngle(0);
  }

  public void servoUp() {
    intakeServo.setAngle(180);
  }

  public void setShooterSpeed(double speed) {
    shooterMotor.set(speed);
  }
  
  public void setFeederSpeed(double speed) {
    shooterFeedMotor.set(speed);
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
