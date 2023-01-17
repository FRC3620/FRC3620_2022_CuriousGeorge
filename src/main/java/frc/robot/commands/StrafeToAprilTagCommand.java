// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AprilTagVision;
import frc.robot.subsystems.DriveSubsystem;

public class StrafeToAprilTagCommand extends CommandBase {
DriveSubsystem driveSubsystem;
AprilTagVision aprilTagVision;
Double distanceToX = AprilTagVision.targetOneX;
double targetSpeed;

  public StrafeToAprilTagCommand (DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set the wheels to strafe here
    driveSubsystem.setWheelsToStrafe(0);
    // driveSubsystem.setWheelsToStrafe(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distanceToX = AprilTagVision.targetOneX;
    if(distanceToX != null)
    {
      targetSpeed = -(1.7*(distanceToX - 0.25));
    }
    else
    {
      targetSpeed = 0;
    }

    if(targetSpeed > -0.2 && targetSpeed < 0.2)
    {
      if (targetSpeed < 0){
        targetSpeed = -0.2;
      }
      else{
        targetSpeed = 0.2;
      }
    }

    // watch the target, and call strafeWidways as needed
    if(distanceToX == null || distanceToX > 0.23 && distanceToX < 0.27)
    {
      driveSubsystem.stopDrive();
      //do nothing
    }
    else
    {
      driveSubsystem.strafeSideways(targetSpeed);
    }
    //right
    /*else if(distanceToX > 0.25)
    {
      driveSubsystem.strafeSideways(0.2);

    }
    //left
    else
    {
      driveSubsystem.strafeSideways(-0.2);
    }*/
    // driveSubsystem.strafeSideways(0);
    SmartDashboard.putNumber("targetSpeed", targetSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // make sure the drive is stopped here!
    driveSubsystem.stopDrive();
    // driveSubsystem.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    {
    return false;
    }
  }
}


//:D