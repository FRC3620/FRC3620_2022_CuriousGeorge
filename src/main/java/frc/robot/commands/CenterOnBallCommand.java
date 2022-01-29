// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class CenterOnBallCommand extends CommandBase {
  /** Creates a new CenterOnBallCommand. */
  
  private DriveSubsystem driveSubsystem;

  static double BALL_LOCATION_TOLERANCE = 0.02;
  static double MAX_ROTATE_SPEED=0.6;
  static double MIN_ROTATE_SPEED=0.3;

  //Set Up Network Tables
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable networkTable = inst.getTable("V/Target");
  private NetworkTableEntry targetX = networkTable.getEntry("target.x");

  
  public CenterOnBallCommand(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    driveSubsystem.setForcedManualModeTrue();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double spinX = 0;
    double targetXFromNetTable = targetX.getDouble(0.5);

    if(targetX.getDouble(0)<0){
      // Vision doesn't see a ball - set spin to MAX_ROTATE_SPEED
      spinX = MAX_ROTATE_SPEED;
    } else {

      // Vision sees a ball; set power proportional to distanve to travel
      spinX = MIN_ROTATE_SPEED + (MAX_ROTATE_SPEED - MIN_ROTATE_SPEED)*(
        (Math.abs(targetXFromNetTable - 0.5))*2);

      if(targetXFromNetTable<0.5){spinX=-spinX;}
      
        
    }

    SmartDashboard.putNumber("CenterOnBall.targetX", targetXFromNetTable);
    SmartDashboard.putNumber("CenterOnBall.spinSpeed",spinX);

    driveSubsystem.autoDrive(0,0,spinX);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    driveSubsystem.teleOpDrive(0,0,0);
    driveSubsystem.setForcedManualModeFalse();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {


    // This method should return true when the the x-value from the network table reads 0.5 +/- TOLERANCE
    if (Math.abs(targetX.getDouble(0.5)-0.5) <= BALL_LOCATION_TOLERANCE){
      return true;
    }

    return false;
  }
}
