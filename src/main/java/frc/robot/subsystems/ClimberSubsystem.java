// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  private final CANSparkMax climbMotor1 = new CANSparkMax(ClimberConstants.kClimber1MotorPort,CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax climbMotor2 = new CANSparkMax(ClimberConstants.kClimber2MotorPort,CANSparkMax.MotorType.kBrushless);


  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}