// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.usfirst.frc3620.misc.RobotMode;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.miscellaneous.CANSparkMaxSendable;

public class ClimberSubsystem extends SubsystemBase {
  DigitalInput climberStationaryHookContact = RobotContainer.climberStationaryHookContact; 
  CANSparkMaxSendable climberExtentionMotor = RobotContainer.climberExtentionMotor; 
  Solenoid climberArmTilt = RobotContainer.climberArmTilt;
  boolean encoderIsValid = true;  // set this back later
  Timer calibrationTimer;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    if(climberExtentionMotor != null) {
      SendableRegistry.addLW(climberExtentionMotor, getName(), "climber motor");
      climberExtentionMotor.setSelectedSensorPosition(0);
    }
    SendableRegistry.addLW(climberStationaryHookContact, getName(), "climber contact switch");
  }

  @Override
  public void periodic() {

    if (climberExtentionMotor != null) { 
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
      SmartDashboard.putNumber("climber.current", climberExtentionMotor.getStatorCurrent());
    }
  }

  public boolean doesStationaryHookHaveBar() {
    boolean rv = !climberStationaryHookContact.get(); 
    return rv; 
  }

  /**
   * Return the position of the climber extension
   * @return position (expressed in something besides furlongs)
   */

  public double getShaftPosition() {
    if (climberExtentionMotor != null) { 
      double rv = climberExtentionMotor.getSelectedSensorPosition();
      rv = rv/2048;
      return rv; 
    }
    else {
      return 0; 
    }
  }

  public void spinClimberExtentionMotor(double speed) {
    if (climberExtentionMotor != null) { 
      climberExtentionMotor.set(ControlMode.PercentOutput, speed);
    }
  }

  public void climberArmTiltIn() {
    if (climberArmTilt != null) { 
      climberArmTilt.set(false);
    }
  }

  public void climberArmTiltOut() {
    if (climberArmTilt != null) { 
      climberArmTilt.set(true);
    }
  }


}