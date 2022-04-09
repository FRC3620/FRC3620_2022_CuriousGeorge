// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class XParallelDeadlineGroup extends ParallelDeadlineGroup {
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);
  /** Creates a new XParallelDeadlineGroup. */
  public XParallelDeadlineGroup(Command c1, Command... commands) {
    super(c1, commands);
    logger.info ("requirements for {}: {}", toString(), getRequirements());
  }
}
