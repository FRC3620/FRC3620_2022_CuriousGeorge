// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import frc.robot.RobotContainer;
import frc.robot.commands.IntakeOffCommand;
import frc.robot.miscellaneous.CANSparkMaxSendable;

public class IntakeSubsystem extends SubsystemBase {
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

  boolean logCallers = false;
  
  CANSparkMaxSendable intakeWheelbar = RobotContainer.intakeWheelbar;
  CANSparkMaxSendable intakeBelt = RobotContainer.intakeBelt;

  Command previousCommand = null;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    if (intakeWheelbar != null) {
      SendableRegistry.addLW(intakeWheelbar, getName(), "intake wheelbar");
    }
    if (intakeBelt != null) {
      SendableRegistry.addLW(intakeBelt, getName(), "intake belt");
    }

    setDefaultCommand(new IntakeOffCommand(this));
  }

  @Override
  public void periodic() {
  }

  Double intakeBeltSpeedShootingOverride = null;
  Double intakeWheelBarSpeedShootingOverride = null;

  double intakeBeltCommandedPower = 0.0;

  /**
   * Spin the intake wheel and intake belt.
   * @param speed how fast to spin. positive is inward, negative is outward.
   */
  public void spinIntakeWheelBar(double speed) {
    if (intakeWheelbar != null) {
      if (intakeWheelBarSpeedShootingOverride == null) {
        intakeWheelbar.set(speed);
      }
    }  
  }

  public void spinIntakeBelt(double speed) {
    if (intakeBelt != null) {
      if (intakeBeltSpeedShootingOverride == null) {
        intakeBelt.set(speed);
      }
    }
    intakeBeltCommandedPower = speed;
  }

  public void overrideIntakeBeltForShooting(double speed) {
    if (logCallers) {
      String where = EventLogging.myAndCallersNames();
      logger.info ("{} with {}", where, speed);
    }
    intakeBeltSpeedShootingOverride = speed;
    if (intakeBelt != null) {
        intakeBelt.set(speed);
    }
    intakeBeltCommandedPower = speed;
  }

  public void overrideIntakeWheelBarForShooting(double speed) {
    if (logCallers) {
      String where = EventLogging.myAndCallersNames();
      logger.info ("{} with {}", where, speed);
    }
    intakeBeltSpeedShootingOverride = speed;
    if (intakeBelt != null) {
        intakeWheelbar.set(speed);
    }
  }

  public void clearIntakeShootingOverrides() {
    if (logCallers) {
      String where = EventLogging.myAndCallersNames();
      logger.info ("{} with {}", where);
    }
    intakeBeltSpeedShootingOverride = null;
    intakeWheelBarSpeedShootingOverride = null;
  }

  public double getIntakeBeltPower() {
    if (intakeBelt != null) {
      return intakeBelt.get();
    }
    return 0;
  }

  public double getIntakeWheelbarPower() {
    if (intakeWheelbar != null) {
      return intakeWheelbar.get();
    }
    return 0;
  }

  public double getCommandedIntakeBeltPower() {
    return intakeBeltCommandedPower;
  }
}