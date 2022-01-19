// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {

      //the diameter of the drivetrain from left wheels to right wheels
      public static final double kDriveWidth = 22;
      
      //motor ports
      public static final int kLeftMotor1Port = 3;
      public static final int kLeftMotor2Port = 4;
      public static final int kRightMotor1Port = 1;
      public static final int kRightMotor2Port = 2;
      public static final int kIntakeMotorPort = 5;
      public static final int kShootingMotorPort = 6;

      //encoder constants  
      public static final int[] kLeftEncoderPorts = new int[] {0, 1};
      public static final int[] kRightEncoderPorts = new int[] {2, 3};
      public static final boolean kLeftEncoderReversed = false;
      public static final boolean kRightEncoderReversed = true;

      //left and right
      public static final int kLeft = -1;
      public static final int kRight = 1;

      //max RPM of drive motors
      public static final int kMaxRPM = 5676;
  
      //distance calculation
      public static final int kEncoderCPR = 1024;
      public static final double kGearRatio = 1/10.75;
      public static final double kWheelDiameterInches = 6;
      public static final double kEncoderDistancePerPulse = (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;
      public static final double kMaxRobotSpeed = (kMaxRPM/60) * kGearRatio * (kWheelDiameterInches * Math.PI);
      
      //drive pid
      public static final double kDefaultP = .00038;
      public static final double kDefaultI = .0000011;
      public static final double kDefaultD = .0001;

      //the zone in which I is used in the drive PID controller
      //public static final double kIZone = (0.1 * kMaxRPM);

      //max acceleration of robot, in ft/sec/sec  
      public static final double kMaxAccel = 12;

      //motor power below this is considered negligible and not applied
      public static final double kMinPower = 0.005;

      public static final double kFast = 1.0;
      public static final double kSlow = 0.2;
    }
  
    public static final class HatchConstants {
      public static final int kHatchSolenoidModule = 0;
      public static final int[] kHatchSolenoidPorts = new int[] {0, 1};
    }
  
    public static final class AutoConstants {
      public static final double kAutoDriveDistanceInches = 60;
      public static final double kAutoBackupDistanceInches = 20;
      public static final double kAutoDriveSpeed = 0.5;
    }
  
    public static final class OIConstants {
      public static final int kDriverControllerPort = 1;
      public static final int kButtonIntakeOn = 6;
      public static final int kButtonIntakeOff = 5;
      //public static final int kButtonIntakeReverse = 5;
      //public static final int kButtonLaunchIntake = 6;
      public static final int kButtonFastGear = 3;
      public static final int kButtonSlowGear = 2;

    }
  }
