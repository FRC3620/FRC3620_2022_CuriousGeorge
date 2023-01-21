// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AprilTagVision;
import frc.robot.subsystems.DriveSubsystem;

public class LocateAprilTagCommand extends CommandBase {
boolean end = false;
boolean iAmDriving =false;
DriveSubsystem driveSubsystem;
AprilTagVision aprilTagVision = RobotContainer.apriltagVisionSubsystem;
AutoDriveCommand aprilTagDrive;
double distance;
double angle;
public double speed = 0.3;
double offset = 36;


public LocateAprilTagCommand (DriveSubsystem driveSubsystem) {
  this.driveSubsystem = driveSubsystem;
  // Use addRequirements() here to declare subsystem dependencies.
  //addRequirements(driveSubsystem);  
}


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set the wheels to strafe here
    aprilTagVision.clearTag1Transform();
    iAmDriving = false;
    end = false;
    // driveSubsystem.setWheelsToStrafe(0);
  }

  AutoDriveCommand autoDriveCommand;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if (! iAmDriving) {
    Transform3d atag1transform = aprilTagVision.getTag1Transform();
    if(atag1transform != null)
    {
      double atag1TransformXm = atag1transform.getX();
      double atag1TransformYm = atag1transform.getY();
      double atag1TransformZm = atag1transform.getZ();
      
      double atag1TransformXinch = atag1TransformXm*39.3701;
      double atag1TransformYinch = atag1TransformYm*39.3701;
      double atag1TransformZinch = atag1TransformZm*39.3701;

      SmartDashboard.putNumber("LAT.tag1posex", atag1TransformXinch);
      SmartDashboard.putNumber("LAT.tag1posey", atag1TransformYinch);
      SmartDashboard.putNumber("LAT.tag1posez", atag1TransformZinch);
      distance = Math.sqrt(Math.pow(atag1TransformZinch-offset, 2) + Math.pow(atag1TransformXinch, 2));
      angle = Math.toDegrees (Math.atan(atag1TransformXinch/atag1TransformZinch));
      SmartDashboard.putNumber("LAT.distance", distance);
      SmartDashboard.putNumber("LAT.Strafe Angle", angle);

      iAmDriving = true;
      autoDriveCommand = new AutoDriveCommand(distance, angle, speed, 0, driveSubsystem);
      autoDriveCommand.initialize();
    } 
    } else {
      
      autoDriveCommand.execute();
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // make sure the drive is stopped here!
    driveSubsystem.stopDrive();
    // driveSubsystem.stopDrive();
    if(autoDriveCommand != null)
    {
      autoDriveCommand.end(false);
    }
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    
      if(autoDriveCommand != null)
      {
        if(autoDriveCommand.isFinished())
        {
          return true;
        }
        else
        {
          return false;
        }
      }

      return false;
    
  }
}


//:D