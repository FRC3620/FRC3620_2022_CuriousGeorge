// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ArmSubsystem extends SubsystemBase {
  DoubleSolenoid climberArmTilt = RobotContainer.climberArmTilt;
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    if (climberArmTilt != null) {
      SendableRegistry.addLW(climberArmTilt, getName(), "tilt");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
//THIS DOES LITERALLY NOTHING BUT DON'T DELETE. IT'S IMPORTANT I SWEAR.