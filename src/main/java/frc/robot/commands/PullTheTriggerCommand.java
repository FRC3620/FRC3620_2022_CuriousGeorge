// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this projec
package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.ShooterDecider;
import frc.robot.miscellaneous.ShooterCalculator;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PreShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;

public class PullTheTriggerCommand extends CommandBase {
  Logger logger = EventLogging.getLogger(getClass(), EventLogging.Level.INFO);

  PreShooterSubsystem preShooterSubsystem = RobotContainer.preShooterSubsystem;
  IntakeSubsystem intakeSubsystem = RobotContainer.intakeSubsystem;
  VisionSubsystem visionSubsystem = RobotContainer.visionSubsystem;

  Timer preshooterTimer = new Timer();
  boolean weAreDone = false;
  double timeBetweenShots = 0.0;

  double totalCommandTime;

  ShooterDecider.PewPewData pewPewData = new ShooterDecider.PewPewData();

  public PullTheTriggerCommand() {
    this(1.5);
  }

  public PullTheTriggerCommand(double totalCommandTime) {
    this.totalCommandTime = totalCommandTime;
    // Use addRequirements() here to declare subsystem dependencies.

    // do not require the intake!!!!!!!!
    addRequirements(preShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    preshooterTimer.reset();
    preshooterTimer.start();
    weAreDone = false;

    visionSubsystem.freezeDistance();

    pewPewData.clear();
    ShooterDecider.isShooterUpToSpeed(pewPewData);
    ShooterDecider.isHoodInPosition(pewPewData);
    ShooterDecider.isTurretInPosition(pewPewData);
    pewPewData.fillInVisionData();
    ShooterDecider.logPewPewData(logger, "doing", pewPewData);

    double targetYLocation = visionSubsystem.getTargetYLocation();
    double distance = ShooterCalculator.calcDistanceFromHub(targetYLocation);
    //double rpm = ShooterCalculator.calcMainRPM(distance);
    //SmartDashboard.putString("doug.bar", "" + targetYLocation + " " + rpm);

    if(distance <= 14){
      timeBetweenShots = 0.0;
    } else {
      timeBetweenShots = 0.4;
    }
    // logger.info ("time between shots: {}", timeBetweenShots);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double t = preshooterTimer.get();
    if (t < timeBetweenShots) {
        preShooterSubsystem.preshooterOn(1.0);
        intakeSubsystem.overrideIntakeBeltForShooting(0.0);
        intakeSubsystem.overrideIntakeWheelBarForShooting(0.0);
    } else if (t < totalCommandTime) {
        preShooterSubsystem.preshooterOn(1.0);
        intakeSubsystem.overrideIntakeBeltForShooting(0.4);
        intakeSubsystem.overrideIntakeWheelBarForShooting(0.0);
    } else {
        weAreDone = true;
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      //intakeSubsystem.startPreviousCommand(this);
    }
    intakeSubsystem.clearIntakeShootingOverrides();
    preShooterSubsystem.preshooterOff();
    preshooterTimer.stop();
    visionSubsystem.unfreezeDistance();
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(weAreDone){
      return true;
    }
    return false;
  }
}