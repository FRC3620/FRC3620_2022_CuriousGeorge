// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class IntakeArmSubsystem extends SubsystemBase {
  DoubleSolenoid intakeArm = RobotContainer.intakeArm;
  /** Creates a new IntakeArmSubsystem. */
  public IntakeArmSubsystem() {
    if (intakeArm != null) {
      SendableRegistry.addLW(intakeArm, getName(), "intake arm");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void extendIntakeArm() {
    if (intakeArm != null) {
      intakeArm.set(Value.kForward);
    }
  }

  public void retractIntakeArm() {
    if (intakeArm != null) {
      intakeArm.set(Value.kReverse);
    }
  }

  public void turnOffIntakeArm() {
    if (intakeArm != null) {
      intakeArm.set(Value.kOff);
    }
  }

}
