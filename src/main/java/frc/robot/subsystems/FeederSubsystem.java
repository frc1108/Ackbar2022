/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.RunCommand;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.Constants.*;
/**
 * Add your docs here.
 */
public class FeederSubsystem extends SubsystemBase implements Loggable {

  private WPI_VictorSPX m_feeder = new WPI_VictorSPX(FeederConstants.CAN_ID_Launcher_Intake);

  // Color sensor
  private final I2C.Port i2cPort = I2C.Port.kMXP;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);
  
  private double m_fastInSpeed = 0.7;
  private double m_slowInSpeed = 0.4;
  private double m_slowOutSpeed = -0.6;
  private final double m_feederRPM = -500;

  public FeederSubsystem(){
    stop();
    m_feeder.configFactoryDefault();
    m_feeder.setNeutralMode(NeutralMode.Brake);

    m_feeder.config_kF(0, 0.1225);
    m_feeder.config_kP(0,1);
    m_feeder.config_kI(0,0);
    m_feeder.config_kD(0,0);

    // Config Talon Tach
    final int kTimeoutMs = 30;
    m_feeder.configSelectedFeedbackSensor(FeedbackDevice.Tachometer, 0, kTimeoutMs);
    int edgesPerCycle = 12;
    m_feeder.configPulseWidthPeriod_EdgesPerRot(edgesPerCycle, kTimeoutMs);
    int filterWindowSize = 1;
    m_feeder.configPulseWidthPeriod_FilterWindowSz(filterWindowSize, kTimeoutMs);

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);

    // Default command to stop() 
    this.setDefaultCommand(new RunCommand(() -> stop(), this));
  }

  @Log
  public boolean getRedMatch(){
    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    return (match.color == kRedTarget);
  }

  @Log
  public boolean getBlueMatch(){
    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    return (match.color == kBlueTarget);
  }
 
  @Log.Dial(name = "Launcher RPM", tabName = "Live", max = 200)
  public double getTachRPM(){
      double tachVel_UnitsPer100ms = m_feeder.getSelectedSensorVelocity(0);
      return -1*tachVel_UnitsPer100ms*600/1024;
  }


  public void setFastInSpeed(double fastInSpeed){
    m_fastInSpeed = fastInSpeed;
  }

  public void setSlowOutSpeed(double slowOutSpeed){
    m_slowOutSpeed = slowOutSpeed;
  }

  public void setSlowInSpeed(double slowInSpeed){
    m_slowInSpeed = slowInSpeed;
  }

  public void fastInFeeder(){
    m_feeder.set(m_fastInSpeed);
  }

  public void stop(){
    m_feeder.stopMotor();
  }

  public void slowInFeeder(){
    m_feeder.set(m_slowInSpeed);
  }

  public void slowOutFeeder(){
    m_feeder.set(m_slowOutSpeed);
  }

  public void startPIDLauncher(){      
    m_feeder.set(ControlMode.Velocity,m_feederRPM*1024/600);
  }

}
