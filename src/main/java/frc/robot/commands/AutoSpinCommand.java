package frc.robot.commands;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.miscellaneous.SwerveCalculator;
import frc.robot.subsystems.DriveSubsystem;

public class AutoSpinCommand extends CommandBase {

  private DriveSubsystem driveSubsystem;

  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

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
    EventLogging.commandMessage(logger);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double heading = driveSubsystem.getNavXFixedAngle(); 
    double spinX = pathSpeed;
    driveSubsystem.autoDrive(0, 0, spinX);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.teleOpDrive(0,0,0);
    driveSubsystem.setForcedManualModeFalse();
    EventLogging.commandMessage(logger, interrupted);
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
