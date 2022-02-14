// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.RelativeEncoder;

import org.usfirst.frc3620.misc.RobotMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ClimberSubsystem extends SubsystemBase {
  DigitalInput climberStationaryHookContact = RobotContainer.climberStationaryHookContact; 
  WPI_TalonFX climberExtentionMotor = RobotContainer.climberExtentionMotor; 
  DoubleSolenoid climberArmTilt = RobotContainer.climberArmTilt;
  boolean encoderIsValid = false;
  Timer calibrationTimer;
  

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    climberExtentionMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double climberSpeed = climberExtentionMotor.getSelectedSensorVelocity();
    
    if(Robot.getCurrentRobotMode() == RobotMode.TELEOP || Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS){
      if (!encoderIsValid) {
        spinClimberExtentionMotor(-0.075);

        if (calibrationTimer == null) {
          calibrationTimer = new Timer();
          calibrationTimer.reset();
          calibrationTimer.start();
        } else {
          if (calibrationTimer.get() > 0.5){
            if (Math.abs(climberSpeed) < 20) {
              encoderIsValid = true;
              spinClimberExtentionMotor(0.0);
              climberExtentionMotor.setSelectedSensorPosition(0.0);
            }
          }
        }
      }
    }
    SmartDashboard.putBoolean("climber.doesstationaryhookhavebar", doesStationaryHookHaveBar());
    SmartDashboard.putNumber("climber.encoder", getShaftPosition());
  }

  public boolean doesStationaryHookHaveBar() {
    boolean rv = !climberStationaryHookContact.get(); 
    return rv; 
  }

  /**
   * Return the position of the climber extension
   * @return position (expressed in furlongs)
   */
  public double getShaftPosition() {
    double rv = climberExtentionMotor.getSelectedSensorPosition();
    rv = rv/2048;
    return rv; 
  }

  public void spinClimberExtentionMotor(double speed) {
    climberExtentionMotor.set(ControlMode.PercentOutput, speed);
  }

  public void climberArmTiltIn() {
    climberArmTilt.set(Value.kReverse);
  }

  public void climberArmTiltOut() {
    climberArmTilt.set(Value.kForward);
  }

  public void climberArmTiltOff() {
    climberArmTilt.set(Value.kOff);
  }

  
}


