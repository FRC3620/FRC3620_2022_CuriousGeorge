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

  double intakeBeltCommandedPower = 0.0;

  boolean logIntakeBeltCallers = false;
  boolean logIntakeBeltPowerUnpected = false;
  boolean powerWasMismatched = false;

  void setIntakeBeltPowerInAParanoidManner(double power) {
    if (logIntakeBeltCallers && (power != intakeBeltCommandedPower)) {
      String where = EventLogging.callChain(0, 3);
      logger.info ("intake belt power changed to {} by {}", power, where);
    }
    if (logIntakeBeltPowerUnpected && intakeBelt != null) {
      double currentPower = intakeBelt.get();
      if (currentPower != intakeBeltCommandedPower) {
        if (!powerWasMismatched) {
          String where = EventLogging.callChain(0, 3);
          logger.info("intake belt power was {}, s/b {}", currentPower, intakeBeltCommandedPower);
        }
        powerWasMismatched = true;
      } else {
        if (powerWasMismatched) {
          logger.info ("intake belt power matches again");
        }
        powerWasMismatched = false;
      }
    }
    if (intakeBelt != null) {
      intakeBelt.set(power);
    }
    intakeBeltCommandedPower = power;
  }

  public void spinIntakeBelt(double power) {
    if (intakeBeltSpeedShootingOverride == null) {
      setIntakeBeltPowerInAParanoidManner(power);
    }
  }

  public void overrideIntakeBeltForShooting(double power) {
    intakeBeltSpeedShootingOverride = power;
    setIntakeBeltPowerInAParanoidManner(power);
  }

  boolean logWheelBarCallers = false;
  public void overrideIntakeWheelBarForShooting(double speed) {
    if (logWheelBarCallers) {
      String where = EventLogging.callChain(0, 2);
      logger.info ("{} with {}", where, speed);
    }
    intakeBeltSpeedShootingOverride = speed;
    if (intakeBelt != null) {
        intakeWheelbar.set(speed);
    }
  }

  public void clearIntakeShootingOverrides() {
    if (logIntakeBeltCallers) {
      String where = EventLogging.callChain(0, 2);
      logger.info ("shooting overrides cleared by {}", where);
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