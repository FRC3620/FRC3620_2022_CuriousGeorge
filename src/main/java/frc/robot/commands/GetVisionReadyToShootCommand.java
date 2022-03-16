// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.ShooterDecider;
import frc.robot.ShootingDataLogger;
import frc.robot.miscellaneous.ShooterCalculator;
import frc.robot.subsystems.VisionSubsystem;
import org.usfirst.frc3620.logger.IFastDataLogger;

public class GetVisionReadyToShootCommand extends GetReadyToShootCommand {
  /** Creates a new GetVisionReadyToShootCommand. */
  VisionSubsystem visionSubsystem;
  Timer turretTimer = new Timer();
  IFastDataLogger dataLogger;

  public GetVisionReadyToShootCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    visionSubsystem = RobotContainer.visionSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    turretTimer.reset();
    turretTimer.start();

    boolean shouldDoDataLogging = SmartDashboard.getBoolean("shooter.datalogging.enabled", false);
    if (shouldDoDataLogging) {
      double length = SmartDashboard.getNumber("shooter.datalogging.length", 15);
      dataLogger = ShootingDataLogger.getShootingDataLogger("vision_shooter_m", length);
      dataLogger.start();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
    if (turretTimer.get() > 0.3){
      //seconds
      setUpStuffToShoot();
      turretTimer.reset(); 
    }
  }

  public void setUpStuffToShoot(){
    double currentTurretPosition = turretSubsystem.getCurrentTurretPosition();
    if (visionSubsystem.isTargetFound()){
      double targetXDegrees = visionSubsystem.getTargetXDegrees();
      double newTurretPosition = targetXDegrees + currentTurretPosition;
      turretSubsystem.setTurretPosition(newTurretPosition);

      double targetYLocation = visionSubsystem.getTargetYLocation();
      double targetDistance = ShooterCalculator.calcDistanceFromHub(targetYLocation);
      double targetRPM = ShooterCalculator.calcMainRPM(targetDistance);
      double targetHood = ShooterCalculator.calcHoodPosition(targetDistance);

      shooterSubsystem.setMainRPM(targetRPM);
      shooterSubsystem.setBackRPM(ShooterCalculator.calculateBackspinRPM(targetRPM));
      shooterSubsystem.setHoodPositionToDegrees(targetHood);

      pewPewData.fillInVisionXDegress(targetXDegrees);
      pewPewData.fillInVisionYLocation(targetYLocation);
      pewPewData.fillInVisionDistance(targetDistance);
    }
  }

  public void end(boolean interrupted) {
    super.end(interrupted);
    if (dataLogger != null) {
      dataLogger.done();
      dataLogger = null;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean ready = super.isFinished();
    if (! visionSubsystem.isTargetCentered()) {
      ready = false;
    }
    ShooterDecider.showReady(ready);
    return ready;
  }
}