// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class SearchForTargetCommand extends CommandBase {

  private VisionSubsystem visionSubsystem;
  private DriveSubsystem driveSubsystem;
  private Logger logger;

  private double initialPositionRightFront;
  private double initialPositionLeftFront;
  private double initialPositionRightBack;
  private double initialPositionLeftBack;
  private double distanceTravelled;
  private double desiredDistance;

  private Timer timer;

  private IAutonomousLogger autonomousLogger;
  private String legName;

  static double BALL_LOCATION_TOLERANCE = 0.02;

  double targetX = 0;
  static double xToDegrees = 0;

  double distance;
  double heading;
  double strafeAngle;
  double speed;
  double ballY;
  double targetDistance;

  AutoDriveCommand driveCommand;

  /** Creates a new SearchForTargetCommand. */
  public SearchForTargetCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.visionSubsystem = visionSubsystem;
    this.driveSubsystem = driveSubsystem;

    this.legName = legName;
    this.autonomousLogger = autonomousLogger;

    this.timer = new Timer();

    SmartDashboard.putBoolean("SearchForTargetCommand.running", false);
    logger = EventLogging.getLogger(SearchForTargetCommand.class, Level.INFO);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("SearchForTargetCommand.running", true);

    targetX = visionSubsystem.getBallXLocation();
    xToDegrees = ((targetX * 100) - 50);

    distance = 5.0;
    heading = driveSubsystem.getTargetHeading(); // should really be current heading, do we need to add a method
    strafeAngle = xToDegrees;
    speed = 0.3; // don't you dare, Grace

    driveSubsystem.setForcedManualModeTrue();

    ballY = visionSubsystem.getBallYLocation();

    targetDistance = (600.6 * ballY * ballY + -216.4 * ballY + 50.71) + 0; 

    //driveCommand = new AutoDriveCommand(distance, strafeAngle, speed, heading, driveSubsystem);

    //driveCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //driveSubsystem.autoDrive(strafeAngle, speed, 0);
    driveCommand = new AutoDriveCommand(targetDistance, strafeAngle, speed, heading, driveSubsystem);
    driveCommand.schedule();

    SmartDashboard.putNumber("SearchForTargetCommand.xToDegrees", xToDegrees);
    SmartDashboard.putNumber("SearchForTargetCommand.ballY", ballY);
    SmartDashboard.putNumber("SearchForTargetCommand.targetDistance", targetDistance);

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("SearchForTargetCommand.running", false);

    driveSubsystem.teleOpDrive(0,0,0);
    driveSubsystem.setForcedManualModeFalse();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return driveCommand.isDone();
    return false;
  }
}
