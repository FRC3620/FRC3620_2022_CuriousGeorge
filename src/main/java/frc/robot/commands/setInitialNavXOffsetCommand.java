// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.print.attribute.standard.MediaSize.NA;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class setInitialNavXOffsetCommand extends InstantCommand {
  DriveSubsystem m_DriveSubsystem;
  double NavXOffsetAngle;
  public setInitialNavXOffsetCommand(DriveSubsystem driveSubsystem, double OffsetAngle) {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    m_DriveSubsystem = driveSubsystem;
    NavXOffsetAngle = OffsetAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DriveSubsystem.setNavXOffset(NavXOffsetAngle);
    m_DriveSubsystem.resetNavX();
    m_DriveSubsystem.setTargetHeading(m_DriveSubsystem.getNavXFixedAngle());
    
  }
}