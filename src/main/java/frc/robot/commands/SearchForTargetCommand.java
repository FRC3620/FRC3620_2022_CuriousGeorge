// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;

import javax.lang.model.util.ElementScanner6;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import pabeles.concurrency.ConcurrencyOps.Reset;

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
  double currentNavX;

  AutoDriveCommand driveCommand;
  ResetNavXCommand navXCommand;

  boolean isTargetFound;
  boolean commandIsFinished;

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

    commandIsFinished = false;

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetX = visionSubsystem.getBallXLocation(); 
    ballY = visionSubsystem.getBallYLocation();

    if (targetX > 0.01 && ballY > 0.01) {
      currentNavX = driveSubsystem.getNavXFixedAngle();


      commandIsFinished = true;

      xToDegrees = ((targetX * 100) - 50);

      distance = 3.775 * Math.exp(6.214*ballY) + 28.11; //(600.6 * ballY * ballY + -216.4 * ballY + 50.71) + 0; 
      heading = currentNavX; //driveSubsystem.getTargetHeading(); // should really be current heading, do we need to add a method
      strafeAngle = xToDegrees + currentNavX; 
      speed = 0.3; // don't you dare, Grace

      driveCommand = new AutoDriveCommand(distance, strafeAngle, speed, heading, driveSubsystem);
      driveCommand.schedule();

      
    SmartDashboard.putNumber("SearchForTargetCommand.xToDegrees", xToDegrees);
    SmartDashboard.putNumber("SearchForTargetCommand.ballY", ballY);
    SmartDashboard.putNumber("SearchForTargetCommand.ballX", targetX);
    SmartDashboard.putNumber("SearchForTargetCommand.targetDistance", distance);
    SmartDashboard.putNumber("SearchForTargetCommand.currentNavX", currentNavX);
    SmartDashboard.putBoolean("SearchForTargetCommand.CommandisFinished", commandIsFinished);

    
    }

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("SearchForTargetCommand.running", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(commandIsFinished == true) {
      return true;
    } else {
      return false;
    }
  }
}
