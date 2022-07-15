// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class RotateToTargetCommand extends CommandBase {
  /** Creates a new RotateToTargetCommand. */
  private DriveSubsystem driveSubsystem;
  private Logger logger;

  static double BALL_LOCATION_TOLERANCE = 0.02;
  static double MAX_ROTATE_SPEED=0.4;
  static double MIN_ROTATE_SPEED=0.4;

  double xToDegrees = 0;
  //xToDegrees = SearchForTargetCommand.xToDegrees;
  
  public RotateToTargetCommand(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    SmartDashboard.putBoolean("RotateToTargetCommand.running", false);
    logger = EventLogging.getLogger(InstantCenterOnBallCommand.class, Level.INFO);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("RotateToTargetCommand.running", true);
    xToDegrees = SearchForTargetCommand.xToDegrees;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
