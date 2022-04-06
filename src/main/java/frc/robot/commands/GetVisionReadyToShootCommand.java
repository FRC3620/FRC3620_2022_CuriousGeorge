// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.ShooterDecider;
import frc.robot.ShootingDataLogger;
import frc.robot.miscellaneous.ShooterCalculator;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.RumbleSubsystem.Hand;

import org.usfirst.frc3620.logger.IFastDataLogger;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RumbleSubsystem;

public class GetVisionReadyToShootCommand extends GetReadyToShootCommand {
  /** Creates a new GetVisionReadyToShootCommand. */
  VisionSubsystem visionSubsystem;
  DriveSubsystem driveSubsystem;
  RumbleSubsystem driverRumbleSubsystem;

  boolean logOkToShootMessages = false;
  
  Timer turretTimer = new Timer();
  IFastDataLogger dataLogger;
  boolean okToShoot;

  public GetVisionReadyToShootCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    visionSubsystem = RobotContainer.visionSubsystem;
    driveSubsystem = RobotContainer.driveSubsystem;
    driverRumbleSubsystem = RobotContainer.driverRumbleSubsystem;
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

    okToShoot = everythingIsReady();
    if (logOkToShootMessages) {
      logger.info("okToShoot 1 {}", okToShoot);
    }
    if (!visionSubsystem.isTargetCentered()) {
      okToShoot = false;
      if (logOkToShootMessages) {
        logger.info("okToShoot 2a {} {}", okToShoot, visionSubsystem.getTargetXDegrees());
      }
    } else {
      if (logOkToShootMessages) {
        logger.info("okToShoot 2b {} {}", okToShoot, visionSubsystem.getTargetXDegrees());
      }
    }
    ShooterDecider.showReady(okToShoot);

    boolean shouldRumble = okToShoot && driveSubsystem.areWeStopped() && DriverStation.isTeleopEnabled();
    SmartDashboard.putBoolean("should rumble", shouldRumble);
    if(shouldRumble){
      driverRumbleSubsystem.setRumble(Hand.RIGHT, 0.2);
    } else {
      driverRumbleSubsystem.clearRumble();
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
      double targetHood = ShooterCalculator.calcHoodAngle(targetDistance);

      SmartDashboard.putNumber("doug.y", targetYLocation);
      SmartDashboard.putNumber("doug.distance", targetDistance);
      SmartDashboard.putNumber("doug.main.rpm", targetRPM);
      SmartDashboard.putNumber("doug.hood", targetHood);

      shooterSubsystem.setMainRPM(targetRPM);
      shooterSubsystem.setBackRPM(ShooterCalculator.calculateBackspinRPM(targetRPM));
      shooterSubsystem.setHoodPositionToDegrees(targetHood);
    }
  }

  public void end(boolean interrupted) {
    pewPewData.fillInVisionData();
    super.end(interrupted);
    if (dataLogger != null) {
      dataLogger.done();
      dataLogger = null;
      driverRumbleSubsystem.clearRumble();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}