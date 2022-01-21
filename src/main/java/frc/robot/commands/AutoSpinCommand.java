package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.miscellaneous.SwerveCalculator;
import frc.robot.subsystems.DriveSubsystem;

public class AutoSpinCommand extends CommandBase {

  private DriveSubsystem driveSubsystem;

  private double desiredHeading;
  private double pathSpeed;

  public AutoSpinCommand(double speed, double heading, DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);

    desiredHeading = heading;
    pathSpeed = speed;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.setForcedManualModeTrue();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double heading = driveSubsystem.getNavXFixedAngle(); 
    double spinX = - pathSpeed;
    driveSubsystem.timedDrive(0, 0, spinX);
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
    double diff = SwerveCalculator.calculateAngleDifference(driveSubsystem.getNavXFixedAngle(), desiredHeading);
    if(Math.abs(diff) < 5){
      return true;
    }
    return false;
  }
}
