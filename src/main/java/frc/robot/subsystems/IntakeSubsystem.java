// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Method;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class IntakeSubsystem extends SubsystemBase {
  CANSparkMax intakeWheelbar = RobotContainer.intakeWheelbar;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  /**
   * Spin the intake wheel and intake belt.
   * @param speed how fast to spin. positive is inward, negative is outward.
   */
  public void spinIntakeMotors(double speed) {
    if (intakeWheelbar != null) {
      intakeWheelbar.set(speed);
    }
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
