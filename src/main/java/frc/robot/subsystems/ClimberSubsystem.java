// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

import org.usfirst.frc3620.misc.RobotMode;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.miscellaneous.CANSparkMaxSendable;

public class ClimberSubsystem extends SubsystemBase {
  DigitalInput climberStationaryHookContact = RobotContainer.climberStationaryHookContact; 
  CANSparkMaxSendable climberExtentionMotor = RobotContainer.climberExtentionMotor; 
  RelativeEncoder climberEncoder;
  boolean encoderIsValid = false;
  Timer calibrationTimer;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    if(climberExtentionMotor != null) {
      SendableRegistry.addLW(climberExtentionMotor, getName(), "climber motor");
      climberEncoder = climberExtentionMotor.getEncoder();
    }
    SendableRegistry.addLW(climberStationaryHookContact, getName(), "climber contact switch");
  }

  @Override
  public void periodic() {

    if (climberExtentionMotor != null) { 
      double climberSpeed = climberEncoder.getVelocity();  // motor revolutions per minute
      
      if(Robot.getCurrentRobotMode() == RobotMode.TELEOP || Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS){
        if (!encoderIsValid) {
          spinClimberExtentionMotor(-0.075);

          if (calibrationTimer == null) {
            calibrationTimer = new Timer();
            calibrationTimer.reset();
            calibrationTimer.start();
          } else {
            if (calibrationTimer.get() > 0.5) {
              if (Math.abs(climberSpeed) < 0.1) {
                encoderIsValid = true;
                spinClimberExtentionMotor(0.0);
                climberEncoder.setPosition(0.0);
              }
            }
          }
        } else {
          if(RobotContainer.getOperatorJoystickRightY()<0 && getClimberExtensionInInches()<=0.25) {
            spinClimberExtentionMotor(0.0);
          } else if(RobotContainer.getOperatorJoystickRightY()>0 && getClimberExtensionInInches()>18.5) {
            spinClimberExtentionMotor(0.0);
          } else if(RobotContainer.getOperatorJoystickRightY()<0 && getClimberExtensionInInches()<=5) {
            spinClimberExtentionMotor(-0.2);
          } else {
            spinClimberExtentionMotor(RobotContainer.getOperatorJoystickRightY());
          }
        }
      } else {
        calibrationTimer = null; // start over
      }
      SmartDashboard.putBoolean("climber.doesstationaryhookhavebar", doesStationaryHookHaveBar());
      SmartDashboard.putNumber("climber.encoder",climberEncoder.getPosition());
      SmartDashboard.putNumber("climber.extension", getClimberExtensionInInches());
      SmartDashboard.putNumber("climber.current", climberExtentionMotor.getOutputCurrent());
      SmartDashboard.putNumber("climber.power", climberExtentionMotor.get());
      SmartDashboard.putBoolean("climber.encoder_is_valid", encoderIsValid);
    }
  }

  public double getClimberExtensionInInches() {
    double rv = climberEncoder.getPosition() * 3.14159 / 20;
    return rv;
  }

  public boolean doesStationaryHookHaveBar() {
    boolean rv = !climberStationaryHookContact.get(); 
    return rv; 
  }

  public void spinClimberExtentionMotor(double speed) {
    if (climberExtentionMotor != null) { 
      climberExtentionMotor.set(speed);
    }
  }
}