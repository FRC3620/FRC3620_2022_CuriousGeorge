// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.slf4j.Logger;

import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ArmSubsystem extends SubsystemBase {
  
	Logger logger = EventLogging.getLogger(getClass(), Level.INFO);
  Solenoid climberArmTilt = RobotContainer.climberArmTilt;
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    if (climberArmTilt != null) {
      logger.info("have climberArmTilt");
      SendableRegistry.addLW(climberArmTilt, getName(), "tilt");
    } else {
      logger.info("missing climberArmTilt");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
//THIS DOES LITERALLY NOTHING BUT DON'T DELETE. IT'S IMPORTANT I SWEAR.