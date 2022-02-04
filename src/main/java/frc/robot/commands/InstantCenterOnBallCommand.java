// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.miscellaneous.DriveVectors;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class InstantCenterOnBallCommand extends InstantCommand {
  private DriveSubsystem driveSubsystem;
  private VisionSubSystem visionSubsystem;

  static double BALL_LOCATION_TOLERANCE = 0.02;
  static double MAX_ROTATE_SPEED=0.4;
  static double MIN_ROTATE_SPEED=0.3;

  double targetX = 0;
  double startNavX = 0;
  //Set Up Network Tables
  //private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  //private NetworkTable networkTable = inst.getTable("V/Target");
  //private NetworkTableEntry targetX = networkTable.getEntry("target.x");

  
  public InstantCenterOnBallCommand(DriveSubsystem driveSubsystem, VisionSubSystem visionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    SmartDashboard.putBoolean("CenterOnBall.running", false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    driveSubsystem.setForcedManualModeTrue();
    SmartDashboard.putBoolean("CenterOnBall.running", true);

    double spinX = 0;
    double spinDegrees = 0;
    double spinHeading = 0;
    
    targetX = visionSubsystem.getBallXLocation();
    startNavX = driveSubsystem.getNavXFixedAngle();

    if(targetX < 0){
      // Vision doesn't see a ball - set spin to MAX_ROTATE_SPEED
      spinX = MAX_ROTATE_SPEED;
      //spinX = 0;
    } else {

      // Vision sees a ball; set power proportional to distanve to travel
      spinDegrees = (Math.abs((targetX - 0.5)/0.0825)*5);

      spinX = MIN_ROTATE_SPEED + (MAX_ROTATE_SPEED - MIN_ROTATE_SPEED)*(
        (Math.abs(targetX - 0.5))*2);

      spinHeading = startNavX + spinDegrees;

      if(targetX<0.5){
          spinX=-spinX;
          spinDegrees=-spinDegrees;
          spinHeading = startNavX - spinDegrees;
        }     
    }

    SmartDashboard.putNumber("CenterOnBall.targetX", targetX);
    SmartDashboard.putNumber("CenterOnBall.spinSpeed",spinX);
    SmartDashboard.putNumber("CenterOnBall.spinDegrees", spinDegrees);

    driveSubsystem.autoDrive(spinHeading, 0, spinX);
  }
}
