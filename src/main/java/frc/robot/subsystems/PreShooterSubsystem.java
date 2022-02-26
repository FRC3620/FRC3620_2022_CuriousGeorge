// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.miscellaneous.MotorStatus;

public class PreShooterSubsystem extends SubsystemBase {
  /** Creates a new PreSubshooterSubsystem. */

  // Set up Motor
  static CANSparkMax preshooter = RobotContainer.shooterSubsystemPreshooter;

  //preshooter FPID Values
  private final double preshooter_FVelocity = 0.0495;//.0456
  private final double preshooter_PVelocity = 0.1; //.45
  private final double preshooter_IVelocity = 0.00; //0.0000001
  private final double preshooter_DVelocity = 0;  //7.5


  public PreShooterSubsystem() {

    if (preshooter != null) {
      SparkMaxPIDController preshooterPid = preshooter.getPIDController();
      preshooterPid.setFF(0);
    }

  }
  
  MotorStatus s_preshooter = new MotorStatus("preshooter");

  public MotorStatus getPreshooterStatus() {
    return s_preshooter;
  }

  // etc etc for rest of PID
  public void setPreshooterRPM(double r) {
    ShooterSubsystem.setRpm(preshooter, r, s_preshooter);
  }
  
  public void preshooterOn(double speed) {
      
    if(preshooter != null) {
       
      preshooter.set(speed);
      
    }
    
  }
  
  public void preshooterOff() {
    if(preshooter != null) {
      preshooter.set(0);      
    }
  }


  @Override
  public void periodic(){
    // This method will be called once per scheduler run

    s_preshooter.gatherActuals(preshooter,"preshooter");
  
  }



}
