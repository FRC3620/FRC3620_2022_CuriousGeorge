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

public class ShooterSubsystem extends SubsystemBase {
  public final static Logger logger = EventLogging.getLogger(ShooterSubsystem.class, Level.INFO);

  WPI_TalonFX m_top1 = RobotContainer.shooterSubsystemTop1;
  WPI_TalonFX m_top2 = RobotContainer.shooterSubsystemTop2;
  WPI_TalonFX m_back = RobotContainer.shooterSubsystemBackShooter;
  private final CANSparkMax hoodMotor = RobotContainer.shooterSubsystemHoodMax;
  RelativeEncoder hoodEncoder = RobotContainer.shooterSubsystemHoodEncoder;
  CANSparkMax preshooter = RobotContainer.shooterSubsystemPreshooter;
  private SparkMaxPIDController anglePID;
  private final int kTimeoutMs = 0;
  private final int kVelocitySlotIdx = 0;

  //top FPID Values
  private final double tFVelocity = 0.049; //0.045
  private final double tPVelocity = 0.45; //0.60
  private final double tIVelocity = 0.0; //0.000003
  private final double tDVelocity = 7.75; //7.75

  //bottom FPID Values
  private final double bFVelocity = 0.0495;//.0456
  private final double bPVelocity = 0.1; //.45
  private final double bIVelocity = 0.00;//0.0000001
  private final double bDVelocity = 0;//7.5

  //hood
  private final double hoodP = 0;
  private final double hoodI = 0;
  private final double hoodD = 0;
  private double hoodPosition = 0;

  public ShooterSubsystem() {
    if (m_top1 != null) {
      SendableRegistry.addLW(m_top1, getName(), "top1");
      setupMotor(m_top1);
      m_top1.setInverted(InvertType.InvertMotorOutput);

      //for PID you have to have a sensor to check on so you know the error
      m_top1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kVelocitySlotIdx, kTimeoutMs);

      //set up the topfalcon for using FPID
      m_top1.config_kF(kVelocitySlotIdx, tFVelocity, kTimeoutMs);
      m_top1.config_kP(kVelocitySlotIdx, tPVelocity, kTimeoutMs);
      m_top1.config_kI(kVelocitySlotIdx, tIVelocity, kTimeoutMs);
      m_top1.config_kD(kVelocitySlotIdx, tDVelocity, kTimeoutMs);
    }

    if (m_top2 != null) {
      SendableRegistry.addLW(m_top2, getName(), "top2");
      setupMotor(m_top2);
      
      m_top2.follow(m_top1);
      m_top2.setInverted(InvertType.OpposeMaster);
    }

    if (m_back != null) {
      setupMotor(m_back);
      SendableRegistry.addLW(m_back, getName(), "back");

      //for PID you have to have a sensor to check on so you know the error
      m_back.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kVelocitySlotIdx, kTimeoutMs);

      //set up the bottomfalcon for using FPID
      m_back.config_kF(kVelocitySlotIdx, bFVelocity, kTimeoutMs);
      m_back.config_kP(kVelocitySlotIdx, bPVelocity, kTimeoutMs);
      m_back.config_kI(kVelocitySlotIdx, bIVelocity, kTimeoutMs);
      m_back.config_kD(kVelocitySlotIdx, bDVelocity, kTimeoutMs);
    }

    if (preshooter != null) {
      SparkMaxPIDController preshooterPid = preshooter.getPIDController();

      preshooterPid.setFF(0);
      // etc etc for rest of PID

    }
  }

  void setupMotor(TalonFX m) {
    m.configFactoryDefault();
    m.setInverted(InvertType.None);

    //set max and minium(nominal) speed in percentage output
    m.configNominalOutputForward(0, kTimeoutMs);
    m.configNominalOutputReverse(0, kTimeoutMs);
    m.configPeakOutputForward(+1, kTimeoutMs);
    m.configPeakOutputReverse(-1, kTimeoutMs);
    
    StatorCurrentLimitConfiguration amprage=new StatorCurrentLimitConfiguration(true,40,0,0); 
    m.configStatorCurrentLimit(amprage);
    m.setNeutralMode(NeutralMode.Coast);
  }

  void setRpm(TalonFX m, double r, Status s) {
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
    s.requestedRPM = r;
    s.requestedSensorVelocity = targetVelocity;
    SmartDashboard.putNumber(s.name + ".rpm.target", r);
    SmartDashboard.putNumber(s.name + ".velocity.target", targetVelocity);
  }

  void setRpm(CANSparkMax m, double r, Status s) {
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
    s.requestedRPM = r;
    s.requestedSensorVelocity = targetVelocity;
    SmartDashboard.putNumber(s.name + ".rpm.target", r);
    SmartDashboard.putNumber(s.name + ".velocity.target", targetVelocity);
  }

  public void setTopRPM(double r) {
    setRpm(m_top1, r, s_top);
  }

  public void setBackRPM(double r) {
    setRpm(m_back, r, s_back);
  }

  public void setPreshooterRPM(double r) {
    setRpm(preshooter, r, s_preshooter);
  }

  Status s_top = new Status("top");
  Status s_back = new Status("back");
  Status s_preshooter = new Status("preshooter");

  public Status getTopStatus() {
    return s_top;
  }

  public Status getBackStatus() {
    return s_back;
  }

  public Status getPreshooterStatus() {
    return s_preshooter;
  }

  void gatherActuals(Status s, TalonFX m, String prefix) {
    if (m != null) {
      s.actualSensorVelocity = m.getSelectedSensorVelocity();
      s.actualRPM = s.actualSensorVelocity * 600 / 2048;
      s.statorCurrent = m.getStatorCurrent();
      s.supplyCurrent = m.getSupplyCurrent();
    } else {
      s.actualSensorVelocity = -1;
      s.actualRPM = -1;
      s.statorCurrent = -1;
      s.supplyCurrent = -1;
    }

    SmartDashboard.putNumber(prefix + ".velocity.actual", s.actualSensorVelocity);
    SmartDashboard.putNumber(prefix + ".rpm.actual", s.actualRPM);
    SmartDashboard.putNumber(prefix + ".current.stator", s.statorCurrent);
    SmartDashboard.putNumber(prefix + ".current.supply", s.supplyCurrent);
  }

  void gatherActuals(Status s, CANSparkMax m, String prefix) {
    if (m != null) {
      RelativeEncoder encoder = m.getEncoder();
      s.actualSensorVelocity = encoder.getVelocity();
      s.actualRPM = s.actualSensorVelocity;
      s.statorCurrent = m.getOutputCurrent();
      s.supplyCurrent = -1;
    } else {
      s.actualSensorVelocity = -1;
      s.actualRPM = -1;
      s.statorCurrent = -1;
      s.supplyCurrent = -1;
    }

    SmartDashboard.putNumber(prefix + ".velocity.actual", s.actualSensorVelocity);
    SmartDashboard.putNumber(prefix + ".rpm.actual", s.actualRPM);
    SmartDashboard.putNumber(prefix + ".current.stator", s.statorCurrent);
    SmartDashboard.putNumber(prefix + ".current.supply", s.supplyCurrent);
    SmartDashboard.putNumber("Hood Position", hoodPosition);

    if (hoodMotor != null) {
      anglePID = hoodMotor.getPIDController();
      hoodEncoder = hoodMotor.getEncoder();
      anglePID.setReference(hoodPosition, ControlType.kPosition);
      anglePID.setP(hoodP);
      anglePID.setI(hoodI);
      anglePID.setD(hoodD);
      anglePID.setOutputRange(-0.5, 0.5);
    }
    
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    gatherActuals(s_top, m_top1, "top");
    gatherActuals(s_back, m_back, "back");
    gatherActuals(s_preshooter, preshooter, "preshooter");
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static class Status {
    String name;
    double requestedRPM = -1;
    double requestedSensorVelocity = -1;
    double actualSensorVelocity = -1;
    double actualRPM = -1;
    double statorCurrent = -1;
    double supplyCurrent = -1;

    Status(String _name) {
      this.name = _name;
    }

    public String getName() {
      return name;
    }

    public double getRequestedRPM() {
      return requestedRPM;
    }

    public double getRequestedSensorVelocity() {
      return requestedSensorVelocity;
    }

    public double getActualSensorVelocity() {
      return actualSensorVelocity;
    }

    public double getActualRPM() {
      return actualRPM;
    }

    public double getStatorCurrent() {
      return statorCurrent;
    }

    public double getSupplyCurrent() {
      return supplyCurrent;
    }
  }

}