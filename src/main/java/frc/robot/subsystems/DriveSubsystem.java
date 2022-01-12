package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class DriveSubsystem extends SubsystemBase {

  //define hardware objects
  private final AHRS ahrs = new AHRS(SPI.Port.kMXP);
  private final CANSparkMax motor1L = new CANSparkMax(DriveConstants.kLeftMotor1Port,CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax motor2L = new CANSparkMax(DriveConstants.kLeftMotor2Port,CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax motor1R = new CANSparkMax(DriveConstants.kRightMotor1Port,CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax motor2R = new CANSparkMax(DriveConstants.kRightMotor2Port,CANSparkMax.MotorType.kBrushless);
  private final CANEncoder encoderL = motor1L.getEncoder();
  private final CANEncoder encoderR = motor1R.getEncoder();

  private double PID_P = DriveConstants.kDefaultP;
  private double PID_I = DriveConstants.kDefaultI;
  private double PID_D = DriveConstants.kDefaultD;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    motor1L.setInverted(false);
    motor2L.setInverted(false);
    motor1R.setInverted(true);
    motor2R.setInverted(true);

    //set the back drive motors to follow the front ones
    motor2L.follow(getFrontLeftSparkMax());
    motor2R.follow(getFrontRightSparkMax());

    //setup PID controller
    //motor1L.getPIDController().setIZone(DriveConstants.kIZone);
    //motor1R.getPIDController().setIZone(DriveConstants.kIZone);
    updatePID();
    
  }

  //updated the PID values on the spark maxes
  private void updatePID() {
    motor1L.getPIDController().setP(PID_P);
    motor1R.getPIDController().setP(PID_P);
    motor1L.getPIDController().setI(PID_I);
    motor1R.getPIDController().setI(PID_I);
    motor1L.getPIDController().setD(PID_D);
    motor1R.getPIDController().setD(PID_D);
  }
  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {

    //forward and turning variables for calculation
    double m_fwd = fwd;
    double m_rot = rot;

    //final output power
    double leftMotorOutput = 0;
    double rightMotorOutput = 0;

    leftMotorOutput += m_fwd;
    rightMotorOutput += m_fwd;

    leftMotorOutput += m_rot;
    rightMotorOutput -= m_rot;

    leftMotorOutput = MathUtil.clamp(leftMotorOutput, -1.0, 1.0);
    rightMotorOutput = MathUtil.clamp(rightMotorOutput, -1.0, 1.0);

    setCoast();

    motor1L.getPIDController().setReference(leftMotorOutput * DriveConstants.kMaxRPM , ControlType.kVelocity);
    motor1R.getPIDController().setReference(rightMotorOutput * DriveConstants.kMaxRPM , ControlType.kVelocity);
  }

  public void DriveVelocity(double velocity) {
    //forward and turning variables for calculation
    double m_fwd = velocity;
    System.out.println((m_fwd / DriveConstants.kMaxRobotSpeed) * DriveConstants.kMaxRPM);
    motor1L.getPIDController().setReference((m_fwd / DriveConstants.kMaxRobotSpeed) * DriveConstants.kMaxRPM , ControlType.kVelocity);
    motor1R.getPIDController().setReference((m_fwd / DriveConstants.kMaxRobotSpeed) * DriveConstants.kMaxRPM , ControlType.kVelocity);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    encoderL.setPosition(0);
    encoderR.setPosition(0);
  }
  public void setDistancePerPulse() {
    encoderL.setPositionConversionFactor((DriveConstants.kGearRatio)*Math.PI*DriveConstants.kWheelDiameterInches);
    encoderR.setPositionConversionFactor((DriveConstants.kGearRatio)*Math.PI*DriveConstants.kWheelDiameterInches);
  }


  public void resetGyro() {
    ahrs.reset();
  }

  public void calibrateGyro() {
    ahrs.calibrate();
    while(ahrs.isCalibrating()) {
    }
  }

  /**
   * Gets the average distance of the TWO encoders.
   *
   * @return the average of the TWO encoder readings
   */
  public double getAverageEncoderDistance() {
    return (encoderL.getPosition() + encoderR.getPosition()) / 2.0;
  }

  public double getEncoderLPosition() {
    return(encoderL.getPosition());
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public CANEncoder getLeftEncoder() {
    return encoderL;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public CANEncoder getRightEncoder() {
    return encoderR;
  }

  public CANSparkMax getFrontLeftSparkMax() {
    return motor1L;
  }

  public CANSparkMax getFrontRightSparkMax() {
    return motor1R;
  }

  public CANSparkMax getBackLeftSparkMax() {
    return motor2L;
  }

  public CANSparkMax getBackRightSparkMax() {
    return motor2R;
  }

  public double getGyro() {
    return (ahrs.getAngle());
  }

  public void setBrake(){
    motor1L.setIdleMode(CANSparkMax.IdleMode.kBrake);
    motor1R.setIdleMode(CANSparkMax.IdleMode.kBrake);
    motor2L.setIdleMode(CANSparkMax.IdleMode.kBrake);
    motor2R.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public void setCoast(){
    motor1L.setIdleMode(CANSparkMax.IdleMode.kCoast);
    motor1R.setIdleMode(CANSparkMax.IdleMode.kCoast);
    motor2L.setIdleMode(CANSparkMax.IdleMode.kCoast);
    motor2R.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  public void restorePID_Defaults() {
    PID_P = DriveConstants.kDefaultP;
    PID_I = DriveConstants.kDefaultI;
    PID_D = DriveConstants.kDefaultD;
    updatePID();
  }

  public void setPID_P(double p) {
    PID_P = p;
    updatePID();
  }

  public void setPID_I(double i) {
    PID_I = i;
    updatePID();
  }

  public void setPID_D(double d) {
    PID_D = d;
    updatePID();
  }

  public void resetIAccum() {
    motor1L.getPIDController().setIAccum(0);
    motor1R.getPIDController().setIAccum(0);
  }

  //get the average speed of the motors
  public double getVelocity() {
    return (getRightEncoder().getVelocity() + getRightEncoder().getVelocity()) / 2;
  }

  @Override
    public void periodic() {

      SmartDashboard.putNumber("encoderL", getLeftEncoder().getPosition());
      SmartDashboard.putNumber("encoderR", getRightEncoder().getPosition());

      SmartDashboard.putNumber("Current Speed", getRightEncoder().getVelocity() / DriveConstants.kMaxRPM);

      //if the difference is greater than 180 degrees, add or subtract one from complete rotations *NOT USED WITH ONLY GYRO
      /*if(Math.abs(oldHeading - currentHeading) > 180) {
        if(oldHeading > currentHeading) {
          completeRotations++;
        }else{
          completeRotations--;
        }
      }*/
    }
}