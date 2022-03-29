// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants{
        public static final int kLeft1MotorPort = 2;
        public static final int kLeft2MotorPort = 3;
        public static final int kRight1MotorPort = 4;
        public static final int kRight2MotorPort = 5;

        public static final double kSpeedMultiplier = 1.0;

        public static final int kLeftControllerPort = 0;
        public static final int kRightControllerPort = 1;
        public static final int kShootingControllerPort = 2;

        public static final double kControllerDeadZone = 0.01;

        // Docs at https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/entering-constants.html
        // These need to be changed to fit our robot.
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;
        public static final double kPDriveVel = 8.5;

        // Needs to be calculated correctly
        public static final double kTrackwidthMeters = 0.69;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kDriveGearRatio = 1/10.71;

        // Should be checked
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

    }

    public static final class IntakeConstants{
        public static final int kIntakeMotorPort = 6;
        public static final int kIntakeServoPort = 1;
        public static final int kIntakeLiftMotorPort = 7;

        public static final double kIntakeSpeed = 0.40;
        public static final double kTransportSpeed = -0.3;
        public static final double kReverseIntakeSpeed = -0.25;
        public static final double KReverseTransportSpeed = 0.15;
    }

    public static final class ShooterConstants{
        public static final int kShooterMotorPort = 8;
        public static final int kTransportMotorPort = 9;

        public static final int kShooterServoLPort = 1;
        public static final int kShooterServoRPort = 0;

        public static final double kShooterLowGoalSpeed = -0.23;
        public static final double kShooterHighGoalCloseSpeed = -0.40;
        public static final double kShooterHighGoalFarSpeed = -0.50;
        public static final double kShooterServoSpeed = 0.5;
    }

    public static final class ClimberConstants{
        public static final int kClimber1MotorPort = 10;
        public static final int kClimber2MotorPort = 11;

        public static final double kClimberSpeed = 0.1;
        public static final double kClimberRaisedPosition = 60;
    }

    public static final class VisionConstants{
        public static final int kTargetPipeline = 2;
        public static final int kRedPipeline = 0;
        public static final int kBluePipeline = 1;

        public static final double kTurnSpeed = 0.25;
        public static final double kCameraHorizontalGoal = 0.25;
        public static final double kCameraVerticalGoal = 0.25;
        public static final double kCameraTargetAreaGoal = 0.25;
    }

    public static final class OIConstants{

        //driver 1 controllers
        public static final int kStartIntakeButton = 3;
        public static final int kStopIntakeButton = 4;
        public static final int kStopShooterButton = 3;
        public static final int kStartTransportButton = 1;
        public static final int kStopTransportButton = 1; // stops when released
        public static final int kRunClimberButton = 7;
        public static final int kClimberBrakeButton = 5;
        public static final int kClimberCoastButton = 6;

        //driver 2 controls
        public static final int kStartShooterHighGoalFarButton = 4;
        public static final int kStartShooterHighGoalCloseButton = 2;
        public static final int kStartShooterLowGoalButton = 1;
        public static final int kReverseIntakeButton = 8;

        //public static final int kLiftIntakeButton = 9;
        //public static final int kDropIntakeButton = 10;
        //public static final int kSliderShooterButton = 8;
    }

}