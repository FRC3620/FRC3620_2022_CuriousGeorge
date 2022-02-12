// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.miscellaneous.DriveVectors;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class InstantCenterOnBallCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private VisionSubsystem VisionSubsystem;
  private Logger logger;

  static double BALL_LOCATION_TOLERANCE = 0.02;
  static double MAX_ROTATE_SPEED=0.4;
  static double MIN_ROTATE_SPEED=0.4;

  double targetX = 0;
  double startNavX = 0;
  //Set Up Network Tables
  //private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  //private NetworkTable networkTable = inst.getTable("V/Target");
  //private NetworkTableEntry targetX = networkTable.getEntry("target.x");

  
  public InstantCenterOnBallCommand(DriveSubsystem driveSubsystem, VisionSubsystem VisionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.VisionSubsystem = VisionSubsystem;
    SmartDashboard.putBoolean("CenterOnBall.running", false);
    logger = EventLogging.getLogger(InstantCenterOnBallCommand.class, Level.INFO);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //driveSubsystem.setForcedManualModeTrue();
    SmartDashboard.putBoolean("CenterOnBall.running", true);

    double spinX = 0;
    double spinDegrees = 0;
    double spinHeading = 0;
    
    targetX = VisionSubsystem.getBallXLocation();
    startNavX = driveSubsystem.getNavXFixedAngle();

    if(targetX < 0){
      // Vision doesn't see a ball - set spin to MAX_ROTATE_SPEED
      logger.info ("Target Not Found");
      logger.info("TargetX: {}", targetX);
      //spinX = MAX_ROTATE_SPEED;
      //spinX = 0;
    } else {

      logger.info ("Target Found");
      logger.info("TargetX: {}", targetX);

      // Vision sees a ball; set power proportional to distanve to travel
      spinDegrees = (Math.abs((targetX - 0.5)/0.0825)*5);

      spinX = MIN_ROTATE_SPEED;
      // + (MAX_ROTATE_SPEED - MIN_ROTATE_SPEED)*(
      //  (Math.abs(targetX - 0.5))*2);

      spinHeading = startNavX + spinDegrees;

      if(targetX<0.5){
          //spinX=-spinX;
          spinHeading = startNavX - spinDegrees;
        }     
      SmartDashboard.putNumber("CenterOnBall.targetX", targetX);
      SmartDashboard.putNumber("CenterOnBall.spinDegrees", spinDegrees);
      SmartDashboard.putNumber("CenterOnBall.spinHeading", spinHeading);
    
      driveSubsystem.setTargetHeading(spinHeading);



    }
  }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      double spinX = driveSubsystem.getSpinPower();
  
      driveSubsystem.autoDrive(0, 0, spinX);

      SmartDashboard.putNumber("CenterOnBall.spinSpeed",spinX);
  
  
      
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      driveSubsystem.autoDrive(0,0,0);
      SmartDashboard.putBoolean("CenterOnBall.running", false);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
