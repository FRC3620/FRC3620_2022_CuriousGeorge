// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;

public class SearchForTargetCommand extends CommandBase {

  private VisionSubsystem visionSubsystem;
  private Logger logger;

  static double BALL_LOCATION_TOLERANCE = 0.02;

  double targetX = 0;
  static double xToDegrees = 0;

  /** Creates a new SearchForTargetCommand. */
  public SearchForTargetCommand(VisionSubsystem visionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.visionSubsystem = visionSubsystem;
    SmartDashboard.putBoolean("SearchForTargetCommand.running", false);
    logger = EventLogging.getLogger(SearchForTargetCommand.class, Level.INFO);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("SearchForTargetCommand.running", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    targetX = visionSubsystem.getBallXLocation();

    if(targetX < 0){
      // Vision doesn't see a ball
      logger.info ("Target Not Found");
      logger.info("TargetX: {}", targetX);
    } else {
      logger.info ("Target Found");
      logger.info("TargetX: {}", targetX);

      xToDegrees = ((targetX * 100) - 50);
    }

    SmartDashboard.putNumber("SearchForTargetCommand.xToDegrees", xToDegrees);
    SmartDashboard.putNumber("SearchForTargetCommand.targetX", targetX);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("SearchForTargetCommand.running", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(xToDegrees != -50) {
      return true;
    }
    return false;
  }
}
