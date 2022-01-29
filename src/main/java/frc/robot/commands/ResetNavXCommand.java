// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class ResetNavXCommand extends InstantCommand {
  /** Add your docs here. */
  DriveSubsystem m_DriveSubsystem;
  public ResetNavXCommand(DriveSubsystem driveSubsystem) {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    m_DriveSubsystem = driveSubsystem;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called once when the command executes
  @Override
  public void initialize() {
    m_DriveSubsystem.resetNavX();
    m_DriveSubsystem.setTargetHeading(m_DriveSubsystem.getNavXFixedAngle());
  }
}
