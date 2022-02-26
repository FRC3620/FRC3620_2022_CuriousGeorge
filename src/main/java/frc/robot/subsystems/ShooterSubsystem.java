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

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.miscellaneous.MotorStatus;
import frc.robot.miscellaneous.ShooterCalculator;
import frc.robot.miscellaneous.CANSparkMaxSendable;

public class ShooterSubsystem extends SubsystemBase {
  public final static Logger logger = EventLogging.getLogger(ShooterSubsystem.class, Level.INFO);

  static WPI_TalonFX m_main2 = RobotContainer.shooterSubsystemMainShooter2;
  WPI_TalonFX m_main1 = RobotContainer.shooterSubsystemMainShooter1;
  static WPI_TalonFX m_back = RobotContainer.shooterSubsystemBackSpinShooter;
  CANSparkMaxSendable hoodMotor = RobotContainer.shooterSubsystemHoodMax;
  RelativeEncoder hoodEncoder = RobotContainer.shooterSubsystemHoodEncoder;
  

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
  private final double hoodP = 0;
  private final double hoodI = 0;
  private final double hoodD = 0;
  private double requestedHoodPosition = 0;

  //backspin FPID
  private final double back_FVelocity = 0.0495;//.0456
  private final double back_PVelocity = 0.1; //.45
  private final double back_IVelocity = 0.00;//0.0000001
  private final double back_DVelocity = 0;//7.5


  public ShooterSubsystem() {
    if (m_main2 != null) {
      SendableRegistry.addLW(m_main2, getName(), "top1");
      m_main2.setInverted(InvertType.InvertMotorOutput);

      //for PID you have to have a sensor to check on so you know the error
      m_main2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kVelocitySlotIdx, kTimeoutMs);

      //set up the topfalcon for using FPID
      m_main2.config_kF(kVelocitySlotIdx, mainFVelocity, kTimeoutMs);
      m_main2.config_kP(kVelocitySlotIdx, mainPVelocity, kTimeoutMs);
      m_main2.config_kI(kVelocitySlotIdx, mainIVelocity, kTimeoutMs);
      m_main2.config_kD(kVelocitySlotIdx, mainDVelocity, kTimeoutMs);
    }

    if (m_main1 != null) {
      SendableRegistry.addLW(m_main1, getName(), "top2");
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
      anglePID.setOutputRange(-0.5, 0.5);
    }
  }

  public double calcHoodPosition(double cy) {
    double calcHoodPosition;
    if(cy < 224){
      calcHoodPosition = 3.73187317480733 + 0.0327847309136473*cy +-0.0000114726741759497*cy*cy;
      calcHoodPosition = calcHoodPosition + SmartDashboard.getNumber("manualHoodPosition", 5);
    } else if(cy < 336){
      calcHoodPosition = 3.85000000000 + 0.0369791667*cy + -0.0000325521*cy*cy;
    }else if(cy < 403){
      calcHoodPosition = -28.1396700696 + 0.2136292223*cy + -0.0002749411*cy*cy;
    } else {
      calcHoodPosition = -56.8299016952515 + 0.355106208706275*cy + -0.000449346405275719*cy*cy;
    }
    return 5.0 * calcHoodPosition;
  }

  public double calcMainRPM(double cy) {
    double calcMainRPM = 2650;
    if(cy < 252) {
      calcMainRPM = 4700;
    } else {
      calcMainRPM = 4700;
    }
    return calcMainRPM;
  }

  public void setTopPower(double p) {
    m_main1.set(p);
  }

  void setRpm(TalonFX m, double r, MotorStatus s) {
    double targetVelocity = 0.0;
    if (m != null) {
      if (r == 0) {
        m.set(ControlMode.PercentOutput, 0);
      } else {
        targetVelocity = r * 2048.0 / 600.0;
        m.set(ControlMode.Velocity, targetVelocity);
      }
    }
    //logger.info ("setRpm {} {}", s.name, r);
    s.setRequestedRPM(r);
    s.setRequestedSensorVelocity(targetVelocity);
  }

  public static void setRpm(CANSparkMax m, double r, MotorStatus s) {
    double targetVelocity = 0;
    if (m != null) {
      if (r == 0) {
        m.set(0);
      } else {
        targetVelocity = r;
        m.getPIDController().setReference(r, CANSparkMax.ControlType.kVelocity);
      }
    }
    //logger.info ("setRpm {} {}", s.name, r);
    s.setRequestedRPM(r);
    s.setRequestedSensorVelocity(targetVelocity);
  }

  public void setMainRPM(double r) {
    setRpm(m_main1, r, s_main);

   // setBackRPM(ShooterCalculator.calculateBackspinRPM(r));
  }

  public void setBackRPM(double r) {
    setRpm(m_back, r, s_back);
  }

  

  MotorStatus s_main = new MotorStatus("main");
  MotorStatus s_back = new MotorStatus("back");
  
  public MotorStatus getTopStatus() {
    return s_main;
  }

  public MotorStatus getBackStatus() {
    return s_back;
  }

  public void setPosition(double position) {
    if(position >= 85){
      requestedHoodPosition = 85;
    } else if (position < 0) {
      requestedHoodPosition=0;
    } else {
      requestedHoodPosition=position;
    }   
      hoodMotor.getPIDController().setReference(requestedHoodPosition,CANSparkMax.ControlType.kPosition );

  }
  
  public double getHoodPosition(){
    return hoodMotor.getEncoder().getPosition();
  }


  public void shooterOff(){
    //sets target velocity to zero
    if (m_main2 != null) {
      m_main2.set(ControlMode.PercentOutput, 0);
    }
    if (m_back != null) {
      m_back.set(ControlMode.PercentOutput, 0);
    }
  }
  
  public void shootPID() {
    double targetVelocity = mainShooterRPM * 2048 / 600;
    if (m_main2 != null) {
      setMainRPM(targetVelocity);
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    s_main.gatherActuals(m_main1, "main");
    s_back.gatherActuals(m_back, "back");
    SmartDashboard.putNumber("Hood Position", getHoodPosition());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


}