// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;
import org.usfirst.frc3620.misc.RobotMode;
import org.usfirst.frc3620.misc.XBoxConstants;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.miscellaneous.MotorStatus;
import frc.robot.miscellaneous.ShooterCalculator;
import frc.robot.miscellaneous.CANSparkMaxSendable;

public class ShooterSubsystem extends SubsystemBase {
  public final static Logger logger = EventLogging.getLogger(ShooterSubsystem.class, Level.INFO);

  WPI_TalonFX m_main1 = RobotContainer.shooterSubsystemMainShooter1;
  WPI_TalonFX m_main2 = RobotContainer.shooterSubsystemMainShooter2;
  WPI_TalonFX m_back = RobotContainer.shooterSubsystemBackSpinShooter;

  CANSparkMaxSendable hoodMotor = RobotContainer.shooterSubsystemHoodMax;
  RelativeEncoder hoodEncoder;
  boolean hoodEncoderIsValid = false;
  Timer hoodTimer;

  double mainShooterRPM = 2000;
  
  private SparkMaxPIDController anglePID;
  private final int kTimeoutMs = 0;

  private final int kVelocitySlotIdx = 0;

  //main shooter FPID Values
  private final double mainFVelocity = 0.049; //0.045
  private final double mainPVelocity = 0.45; //0.60
  private final double mainIVelocity = 0.0; //0.000003
  private final double mainDVelocity = 7.75; //7.75

  //hood
  private final double hoodP = 0.1;
  private final double hoodI = 0;
  private final double hoodD = 0;
  private final double maximumHoodPosition = 110;
  private final double minimumHoodPosition = 6;
  private double requestedHoodPosition = 0;

  //backspin FPID
  private final double back_FVelocity = 0.0495;//.0456
  private final double back_PVelocity = 0.1; //.45
  private final double back_IVelocity = 0.00;//0.0000001
  private final double back_DVelocity = 0;//7.5

  private double requestedMainVelocity, requestedBackSpinShooterVelocity;

  public ShooterSubsystem() {
    if (m_main1 != null) {
      SendableRegistry.addLW(m_main1, getName(), "main1");

      //for PID you have to have a sensor to check on so you know the error
      m_main1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kVelocitySlotIdx, kTimeoutMs);

      //set up the topfalcon for using FPID
      m_main1.config_kF(kVelocitySlotIdx, mainFVelocity, kTimeoutMs);
      m_main1.config_kP(kVelocitySlotIdx, mainPVelocity, kTimeoutMs);
      m_main1.config_kI(kVelocitySlotIdx, mainIVelocity, kTimeoutMs);
      m_main1.config_kD(kVelocitySlotIdx, mainDVelocity, kTimeoutMs);
    }

    if (m_main2 != null) {
      SendableRegistry.addLW(m_main2, getName(), "main2");
      m_main2.follow(m_main1);
    }

    if (m_back != null) {
      SendableRegistry.addLW(m_back, getName(), "back");

      //for PID you have to have a sensor to check on so you know the error
      m_back.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kVelocitySlotIdx, kTimeoutMs);

      //set up the backspin for using FPID
      m_back.config_kF(kVelocitySlotIdx, back_FVelocity, kTimeoutMs);
      m_back.config_kP(kVelocitySlotIdx, back_PVelocity, kTimeoutMs);
      m_back.config_kI(kVelocitySlotIdx, back_IVelocity, kTimeoutMs);
      m_back.config_kD(kVelocitySlotIdx, back_DVelocity, kTimeoutMs);
    }
    
    if (hoodMotor != null) {
      SendableRegistry.addLW(hoodMotor, getName(), "hoodMotor");
      anglePID = hoodMotor.getPIDController();
      hoodEncoder = hoodMotor.getEncoder();
      anglePID.setReference(requestedHoodPosition, ControlType.kPosition);
      anglePID.setP(hoodP);
      anglePID.setI(hoodI);
      anglePID.setD(hoodD);
      anglePID.setOutputRange(-0.5, 0.5); //TODO Set back to 0.5 after testing
    }

    //Load "cargo.desireRPM" value in SmartDashboard
    SmartDashboard.putNumber("cargo.desiredRPM", 120.0);
  }

  public void setTopPower(double p) {
    if (m_main1 != null) {
      m_main1.set(p);
      s_main.setRequestedRPM(-1);
      s_main.setRequestedSensorVelocity(-1);
    }
  }

  double setRpm(TalonFX m, double r, MotorStatus s) {
    double targetVelocity = 0.0;
    if (m != null) {
      if (r == 0) {
        m.set(ControlMode.PercentOutput, 0);
      } else {
        targetVelocity = r * 2048.0 / 600.0;
        if (!Double.isNaN(targetVelocity)) {
          // do not do anything; leave the setpoint where it is!
          m.set(ControlMode.Velocity, targetVelocity);
        }
      }
    }
    //logger.info ("setRpm {} {}", s.name, r);
    s.setRequestedRPM(r);
    s.setRequestedSensorVelocity(targetVelocity);
    return targetVelocity;
  }

  public static double setRpm(CANSparkMax m, double r, MotorStatus s) {
    double targetVelocity = 0;
    if (m != null) {
      if (r == 0) {
        m.set(0);
      } else {
        targetVelocity = r;
        if (!Double.isNaN(targetVelocity)) {
          // do not do anything; leave the setpoint where it is!
          m.getPIDController().setReference(r, CANSparkMax.ControlType.kVelocity);
        }
      }
    }
    //logger.info ("setRpm {} {}", s.name, r);
    s.setRequestedRPM(r);
    s.setRequestedSensorVelocity(targetVelocity);
    return targetVelocity;
  }

  public void setMainRPM(double r) {
    requestedMainVelocity = setRpm(m_main1, r, s_main);
  }

  public double getRequestedMainShooterVelocity(){
    return requestedMainVelocity;
  }

  public double getActualMainShooterVelocity(){
    if (m_main1 != null) {
      return m_main1.getSelectedSensorVelocity();
    }
    return -1;
  }

  public void setBackRPM(double r) {
    if (r < 0){
      r = 0;
    }
    requestedBackSpinShooterVelocity = setRpm(m_back, r, s_back);
  }

  public double getRequestedBackSpinShooterVelocity() {
    return requestedBackSpinShooterVelocity;
  }

  public double getActualBackSpinShooterVelocity() {
    if (m_back != null) {
      return m_back.getSelectedSensorVelocity();
    }
    return -1;
  }
  
  MotorStatus s_main = new MotorStatus("main");
  MotorStatus s_back = new MotorStatus("back");
  
  public MotorStatus getTopStatus() {
    return s_main;
  }

  public MotorStatus getBackStatus() {
    return s_back;
  }

  public void setHoodPositionToDegrees(double degrees) {
    SmartDashboard.putNumber("hood.degrees.requested", degrees);
    double calcuated = ShooterSubsystem.calculateHoodRotations(degrees);
    setHoodPositionToRotations(calcuated);
  }

  public void setHoodPositionToHome() {
    setHoodPositionToRotations(0);
  }

  Double requestedHoodPositionDuringCalibration = null;
  void setHoodPositionToRotations(double position) {
    if (Double.isNaN(position)) {
      // do not do anything; leave the position where it is!
    } else if (position >= 27){
      requestedHoodPosition = 27;
    } else if (position < 2) {
      requestedHoodPosition=2;
    } else {
      requestedHoodPosition=position;
    }
    if (hoodEncoderIsValid) {
      if (hoodMotor != null) {
        hoodMotor.getPIDController().setReference(requestedHoodPosition,CANSparkMax.ControlType.kPosition );
      }
    } else {
      requestedHoodPositionDuringCalibration = requestedHoodPosition;
    }
  }

  public void setHoodPower(double power){
    if (hoodMotor != null) {
      hoodMotor.set(power);
      requestedHoodPosition = 0;
    }
  }

  public void resetHoodEncoder() {
    if (hoodEncoder != null) {
      hoodEncoder.setPosition(0);
      requestedHoodPosition = 0;
    }
  }

  public double getHoodPosition(){
    if (hoodMotor != null) {
      return hoodMotor.getEncoder().getPosition();
    } else {
      return requestedHoodPosition;
    }
  }

  public double getRequestedHoodPosition(){
    return requestedHoodPosition;
  }

  public void shooterOff(){
    //sets target velocity to zero
    if (m_main1 != null) {
      m_main1.set(ControlMode.PercentOutput, 0);
    }
    if (m_back != null) {
      m_back.set(ControlMode.PercentOutput, 0);
    }
  }
    
  public void shootPID() {
    double targetVelocity = mainShooterRPM * 2048 / 600;
    if (m_main1 != null) {
      setMainRPM(targetVelocity);
    }
  }
  
  public static double calculateHoodRotations (double angle) {
    return (58.816 - (0.709 * angle));
  }

  Command lastCommand = null;
  
  @Override
  public void periodic() {
    Command currentCommand = getCurrentCommand();
    if (currentCommand != lastCommand) {
      logger.info("new command: {} -> {}", lastCommand, currentCommand);
      lastCommand = currentCommand;
    }

    // This method will be called once per scheduler run
    s_main.gatherActuals(m_main1, "main");
    s_back.gatherActuals(m_back, "back");
    SmartDashboard.putNumber("hood.actual", getHoodPosition());
    SmartDashboard.putBoolean("hood.encoderisvalid", hoodEncoderIsValid);
    SmartDashboard.putNumber("hood.requested.position", requestedHoodPosition);

    if (hoodMotor != null) { 
      double hoodSpeed = hoodEncoder.getVelocity();  // motor revolutions per minute
      
      if(Robot.getCurrentRobotMode() == RobotMode.TELEOP || Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS){
        if (!hoodEncoderIsValid) {
          setHoodPower(-0.02);
          if (hoodTimer == null) {
            hoodTimer = new Timer();
            hoodTimer.reset(); 
            hoodTimer.start();
          } else {
            if (hoodTimer.get() > 0.5){
              if (Math.abs(hoodSpeed) < 0.1) {
                hoodEncoderIsValid = true;
                setHoodPower(0.0);
                resetHoodEncoder();
                if (requestedHoodPositionDuringCalibration != null) {
                  hoodMotor.getPIDController().setReference(requestedHoodPositionDuringCalibration,CANSparkMax.ControlType.kPosition );
                  requestedHoodPositionDuringCalibration = null;
                }
              }
            }
          } 
        } 
      } else {
        hoodTimer = null; // start over
      }
      SmartDashboard.putNumber("hood.applied.power", hoodMotor.getAppliedOutput());
      SmartDashboard.putNumber("hood.output.current", hoodMotor.getOutputCurrent());
    } else {
      // no hoodmotor,so fake it
      hoodEncoderIsValid = true;
    }
  }

  public boolean hoodEncoderIsValid() {
    return hoodEncoderIsValid;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}