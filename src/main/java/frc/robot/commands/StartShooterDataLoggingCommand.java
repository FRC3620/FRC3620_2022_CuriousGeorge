// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.usfirst.frc3620.logger.IFastDataLogger;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.ShootingDataLogger;

public class StartShooterDataLoggingCommand extends InstantCommand {
  String name;
  double length;
  /** Creates a new StartShooterDataLoggingCommand. */
  public StartShooterDataLoggingCommand(String name, double length) {
    this.name = name;
    this.length = length;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IFastDataLogger dataLogger = ShootingDataLogger.getShootingDataLogger(name, length);
    dataLogger.start();
  }
}
