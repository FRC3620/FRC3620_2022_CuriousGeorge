// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

/*import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance; */
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class CenterOnBallCommand extends CommandBase {
  /** Creates a new CenterOnBallCommand. */
  
  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;

  static double BALL_LOCATION_TOLERANCE = 0.02;
  static double MAX_ROTATE_SPEED=0.4;
  static double MIN_ROTATE_SPEED=0.3;

  double targetX = 0;
  //Set Up Network Tables
  //private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  //private NetworkTable networkTable = inst.getTable("V/Target");
  //private NetworkTableEntry targetX = networkTable.getEntry("target.x");

  
  public CenterOnBallCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
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

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double spinX = 0;
    double spinDegrees = 0;
    
    targetX = visionSubsystem.getBallXLocation();

    if(targetX < 0){
      // Vision doesn't see a ball - set spin to MAX_ROTATE_SPEED
      spinX = MAX_ROTATE_SPEED;
      //spinX = 0;
    } else {

      // Vision sees a ball; set power proportional to distance to travel
      spinDegrees = (Math.abs((targetX - 0.5)/0.0825)*5);

      spinX = MIN_ROTATE_SPEED + (MAX_ROTATE_SPEED - MIN_ROTATE_SPEED)*(
        (Math.abs(targetX - 0.5))*2);

      if(targetX<0.5){
          spinX=-spinX;
          spinDegrees=-spinDegrees;
        }     
    }

    SmartDashboard.putNumber("CenterOnBall.targetX", targetX);
    SmartDashboard.putNumber("CenterOnBall.spinSpeed",spinX);
    SmartDashboard.putNumber("CenterOnBall.spinDegrees", spinDegrees);

    driveSubsystem.autoDrive(spinDegrees, 0, spinX);
    //driveSubsystem.twoWheelRotation(spinX);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("CenterOnBall.running", false);

    driveSubsystem.teleOpDrive(0,0,0);
    driveSubsystem.setForcedManualModeFalse();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This method should return true when the the x-value from the network table reads 0.5 +/- TOLERANCE
    if (Math.abs(targetX-0.5) <= BALL_LOCATION_TOLERANCE){
      return true;
    }
    return false;
  }
}
