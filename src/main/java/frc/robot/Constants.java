/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants{

    // CAN IDs for Spark Max drivetrain controllers
    public static final int kLeftMainPort = 1;
    public static final int kLeftFollowPort = 3;
    public static final int kRightMainPort = 2;
    public static final int kRightFollowPort = 4;

    // Physical robot parameters
    public static final int kEncoderCPR = 42;  // NEO motor encoder Counts per revolution
    public static final double kGearRatio = 8.45; // Toughbox mini gear ratio
    public static final double kWheelDiameterMeters = Units.inchesToMeters(6); // Wheel diameter
    // Use sysid angular to determine the best robot wheel width (may not match tape measurer)
    public static final double kTrackwidthMeters = Units.inchesToMeters(20);
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final Boolean kGyroReversed = true; 
    // WPIlib is ccw positive, use to invert gyro match
    // NavX = true, ADIS16470 = false

    // Encoder count conversion on the spark max for NEOs from rotations to SI units 
    public static final double kEncoderDistanceConversionFactor = ((double) (Math.PI*kWheelDiameterMeters)/(kGearRatio));
    public static final double kEncoderVelocityConversionFactor = ((double) (Math.PI*kWheelDiameterMeters)/(60*kGearRatio));

    // Tuning parameters, use sysid to determine values
    public static final double ksVolts = 0.24489; // 0.169
    public static final double kvVoltSecondsPerMeter = 2.2877;
    public static final double kaVoltSecondsSquaredPerMeter = 0.58909;  //0.0435
    public static final double kPDriveVel = 3.2705;  //2.4 8/14 2.24 Tuning to get better PIDF response

    // Log of sysID values
    // Jan17 Ackbar Ks = 0.24489  Kv = 2.2877  Ka = 0.58909  Kp = 3.2705

    // PID turning parameter and closed loop driving paramters. Uncomment to use
    public static final double kTurnP = 0.125; //0.94, 0.125
    public static final double kTurnI = 0.00;
    public static final double kTurnD = 0.0135; //0.04, 0.0085
    public static final double kMinCommand = 0.0;
    public static final double kMaxTurnRateDegPerS = 360;
    public static final double kMaxTurnAccelerationDegPerSSquared = 480;
    public static final double kTurnToleranceDeg = 3; //0.5
    public static final double kTurnRateToleranceDegPerS = 8;
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(8);

    // Ultrasonic sensor parameters
    public static final int kUltrasonicPort = 3;
    public static final double kValueToInches = 0.02482;
  }

  public static final class BallLauncherConstants {
    public static final int CAN_ID_BALL_LAUNCH_LEFT = 7;
    public static final int CAN_ID_BALL_LAUNCH_RIGHT = 8;
  }

  public static final class HopperConstants {
    public static final int UPPER_LIMIT_SWITCH = 1; //upper limit switch
    public static final int LOWER_LIMIT_SWITCH = 2; //lower limit switch
    public static final int CAN_ID_Hopper_Axle = 5; //Axle
  }

  public static final class IntakeConstants {
    public static final double hopperIntakeSpeed = -0.70; //hopper intake speed
    public static final int CAN_ID_Hopper_Intake = 6; //Intake
  }

  public static final class FeederConstants {
    public static final double launcherIntakeSpeed = 1; //middle intake speed
    public static final int CAN_ID_Launcher_Intake = 9; // launcher intake
  }

  public static final class ClimberConstants {
    public static final int CAN_ID_Climber_Motor = 10;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static final class AutoConstants { 
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(8);
    public static final double kMaxAccelerationMetersPerSecondSquared = Units.feetToMeters(8);

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    
    public static final double kmaxCentripetalAccelerationMetersPerSecondSq = 0.03;
    public static final double  kDifferentialDriveKinematicsConstraint = 0.3;
  }
}
