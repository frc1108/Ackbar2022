/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.SPI;

import frc.robot.pantherlib.Trajectory6391;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.kauailabs.navx.frc.AHRS;

import static frc.robot.Constants.*;

import java.io.IOException;
import java.nio.file.Paths;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class DriveSubsystem extends SubsystemBase implements Loggable {
  private final CANSparkMax m_leftMain = new CANSparkMax(DriveConstants.kLeftMainPort, MotorType.kBrushless);
  private final CANSparkMax m_leftFollow = new CANSparkMax(DriveConstants.kLeftFollowPort, MotorType.kBrushless);
  private final CANSparkMax m_rightMain = new CANSparkMax(DriveConstants.kRightMainPort, MotorType.kBrushless);
  private final CANSparkMax m_rightFollow = new CANSparkMax(DriveConstants.kRightFollowPort, MotorType.kBrushless);
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMain,m_rightMain);

  // Wheel velocity PID control for robot
  private final PIDController m_leftPID = new PIDController(DriveConstants.kPDriveVel, 0, 0);
  private final PIDController m_rightPID = new PIDController(DriveConstants.kPDriveVel, 0, 0);
  private final SimpleMotorFeedforward m_leftfeedforward = new SimpleMotorFeedforward(DriveConstants.ksVolts,DriveConstants.kvVoltSecondsPerMeter,DriveConstants.kaVoltSecondsSquaredPerMeter);
  private final SimpleMotorFeedforward m_rightfeedforward = new SimpleMotorFeedforward(DriveConstants.ksVolts,DriveConstants.kvVoltSecondsPerMeter,DriveConstants.kaVoltSecondsSquaredPerMeter);

  public double slewSpeed = 6;  // in units/s
  public double slewTurn = 6;
  private final SlewRateLimiter m_speedSlew = new SlewRateLimiter(slewSpeed);
  private final SlewRateLimiter m_turnSlew = new SlewRateLimiter(slewTurn);

  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;
  private final DifferentialDriveOdometry m_odometry;
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  // private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  // Setup ultrasonic sensor
  private final AnalogInput m_ultrasonic = new AnalogInput(DriveConstants.kUltrasonicPort);

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    // Stops drive motors
    idle();

    // Restores default CANSparkMax settings
    m_leftMain.restoreFactoryDefaults();
    m_leftFollow.restoreFactoryDefaults();
    m_rightMain.restoreFactoryDefaults();
    m_rightFollow.restoreFactoryDefaults();

    m_leftFollow.follow(m_leftMain);
    m_rightFollow.follow(m_rightMain);

    m_leftMain.setInverted(false);
    m_rightMain.setInverted(true);
    
    // Set Idle mode for CANSparkMax (brake)
    m_leftMain.setIdleMode(IdleMode.kBrake);
    m_leftFollow.setIdleMode(IdleMode.kBrake);
    m_rightMain.setIdleMode(IdleMode.kBrake);
    m_rightFollow.setIdleMode(IdleMode.kBrake);
    
    // Set Smart Current Limit for CAN SparkMax
    m_leftMain.setSmartCurrentLimit(40, 60);
    m_leftFollow.setSmartCurrentLimit(40, 60);
    m_rightMain.setSmartCurrentLimit(40, 60);
    m_rightFollow.setSmartCurrentLimit(40, 60);

    // Setup NEO internal encoder to return SI units for odometry
    m_leftEncoder = m_leftMain.getEncoder();
    m_rightEncoder = m_rightMain.getEncoder();
    m_rightEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistanceConversionFactor);
    m_rightEncoder.setVelocityConversionFactor(DriveConstants.kEncoderVelocityConversionFactor);
    m_leftEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistanceConversionFactor);
    m_leftEncoder.setVelocityConversionFactor(DriveConstants.kEncoderVelocityConversionFactor);

    // Burn settings into Spark MAX flash
    m_leftMain.burnFlash();
    m_leftFollow.burnFlash();
    m_rightMain.burnFlash();
    m_rightFollow.burnFlash(); 

    // Set drive deadband and safety 
    m_drive.setDeadband(0.05);
    m_drive.setSafetyEnabled(false);

    // Start robot odometry tracker
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Updating values to logging and odometry, or other periodic updates
   */
  @Override
  public void periodic() {
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
    SmartDashboard.putNumber("Angle",getHeading());
    SmartDashboard.putNumber("Left Dist", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Dist", m_rightEncoder.getPosition());  
  }

  /***** Drivetrain methods
   * idle: set motors to idle
   * setMaxOutput: set drivetrain max speed [Config]
   * arcadeDrive: set output of drive motors with robot speed and rotation
   * tankDriveVolts: set voltage of drive motors directly
   * tankDriveFeedforwardPID: set wheel speed of drive motors closed loop
   */

  public void idle(){
    m_leftMain.stopMotor();
    m_leftFollow.stopMotor();
    m_rightMain.stopMotor();
    m_rightFollow.stopMotor();
  }

  @Config(name="Max Drive Output", defaultValueNumeric = 0.9)
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(m_speedSlew.calculate(-fwd), 0.8*m_turnSlew.calculate(rot));
  }

  public void curvatureDrive(double fwd, double rot, boolean quickTurn) {
    m_drive.curvatureDrive(m_speedSlew.calculate(-fwd), m_turnSlew.calculate(rot), quickTurn);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMain.setVoltage(leftVolts);
    m_rightMain.setVoltage(rightVolts);
    m_drive.feed();
  }

  public void tankDriveWithFeedforwardPID(double leftVelocitySetpoint, double rightVelocitySetpoint) {
      m_leftMain.setVoltage(m_leftfeedforward.calculate(leftVelocitySetpoint)
          + m_leftPID.calculate(m_leftEncoder.getVelocity(), leftVelocitySetpoint));
      m_rightMain.setVoltage(m_rightfeedforward.calculate(rightVelocitySetpoint)
          + m_rightPID.calculate(m_rightEncoder.getVelocity(), rightVelocitySetpoint));
    m_drive.feed();
  } 

  /***** Gyro methods
   * zeroHeading: sets gyro to zero
   * getHeading: returns gyro angle in degrees [Log]
   * getHeadingCW: returns gyro angle in degrees positive clockwise [Log]
   * getTurnRateCW: returns gyro rate in degrees/sec positive clockwise
   */

  public void zeroHeading() {
    m_gyro.reset();
  }

  @Log
  public double getHeading() {
      return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);//gyro is inversed
  }

  @Log
  public double getHeadingCW() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360);
  }

  public double getTurnRateCW() {
    return m_gyro.getRate();
  }

  /***** Encoder methods
  * resetEncoders: sets encoders to zero
  * getAverageEncoderDistance: get combined left and right changes in position
  * getLeftEncoder: get left RelativeEncoder
  * getRightEncoder: get right RelativeEncoder
  */

  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
  }

  public RelativeEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  public RelativeEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  /***** Odometry methods - keep track of robot pose 
   * resetOdometry: Set odometry position with reset encoder and gyro
   * getPose: Get current robot pose
   * getWheelSpeeds: 
  */

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    zeroHeading();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(),m_rightEncoder.getVelocity());
  }

    /***** Trajectory methods - make paths for robot to follow 
   * createCommandForTrajectory:
   * loadTrajectory:
   * generateTrajectory:
   * loadTrajectoryFromFile:
   * generateTrajectoryFromFile:
   * TODO explain these methods
   */
  
  /**
   * Creates a command to follow a Trajectory on the drivetrain.
   * @param trajectory trajectory to follow
   * @return command that will run the trajectory
   */
  public Command createCommandForTrajectory(Trajectory trajectory, Boolean initPose) {
    if (initPose) {
      new InstantCommand(() -> {resetOdometry(trajectory.getInitialPose());});
    }

    resetEncoders();

    RamseteCommand ramseteCommand =  new RamseteCommand(trajectory, this::getPose,
    new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                    DriveConstants.kaVoltSecondsSquaredPerMeter),
    DriveConstants.kDriveKinematics, this::getWheelSpeeds,
    new PIDController(DriveConstants.kPDriveVel, 0, 0),
    new PIDController(DriveConstants.kPDriveVel, 0, 0), this::tankDriveVolts, this);
    return ramseteCommand.andThen(() -> this.tankDriveVolts(0, 0));
  }

  protected static Trajectory loadTrajectory(String trajectoryName) throws IOException {
    return TrajectoryUtil.fromPathweaverJson(
        Filesystem.getDeployDirectory().toPath().resolve(Paths.get("paths", trajectoryName + ".wpilib.json")));
  }

  public Trajectory generateTrajectory(String trajectoryName, TrajectoryConfig config) {
    try {
      var filepath = Filesystem.getDeployDirectory().toPath().resolve(Paths.get("waypoints", trajectoryName));
      return Trajectory6391.fromWaypoints(filepath, config);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load auto trajectory: " + trajectoryName, false);
      return new Trajectory();
    }
  }
  public Trajectory loadTrajectoryFromFile(String filename) {
    try {
      return loadTrajectory(filename);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load auto trajectory: " + filename, false);
      return new Trajectory();
    }
  }

  public Trajectory generateTrajectoryFromFile(String filename) {
      var config = new TrajectoryConfig(1, 3);
      return generateTrajectory(filename, config);
  }

  /***** Other methods
  * getSonarDistanceInches: get ultrasonic sensor distance in inches [Log] 
  */

  @Log(name = "Ultrasonic, in")
  public double getSonarDistanceInches(){
    return m_ultrasonic.getValue()*DriveConstants.kValueToInches;
  }
}