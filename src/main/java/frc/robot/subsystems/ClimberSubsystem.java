// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ClimberSubsystem extends SubsystemBase {
  DigitalInput climberStationaryHookContact = RobotContainer.climberStationaryHookContact; 
  Victor climberExtentionMotor = RobotContainer.climberExtentionMotor; 
  DoubleSolenoid climberArmTilt = RobotContainer.climberArmTilt;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("climber.doesstationaryhookhavebar", doesStationaryHookHaveBar());
  }

  public boolean doesStationaryHookHaveBar() {
    boolean rv = !climberStationaryHookContact.get(); 
    return rv; 
  }

  public void spinClimberExtentionMotor(double speed) {
    climberExtentionMotor.set(speed);
  }

  public void climberArmTiltIn() {
    climberArmTilt.set(Value.kReverse);
  }

  public void climberArmTiltOut() {
    climberArmTilt.set(Value.kForward);
  }

}
