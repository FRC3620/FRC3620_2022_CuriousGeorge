// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

public class AutoShootCommand extends GetVisionReadyToShootCommand {
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

  /** Creates a new AutoShootCommand. */
  public AutoShootCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    EventLogging.commandMessage(logger);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    EventLogging.commandMessage(logger, interrupted);
    if (!interrupted) {
      logPewPewData();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // logOkToShootMessages is defined and set in superclass
    if (logOkToShootMessages) {
      logger.info("okToShoot 3 {}", okToShoot);
    }
    return okToShoot;
  }
}