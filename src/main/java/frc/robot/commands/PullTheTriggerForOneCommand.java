// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this projec
package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.ShooterDecider;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PreShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;

public class PullTheTriggerForOneCommand extends CommandBase {
  Logger logger = EventLogging.getLogger(getClass(), EventLogging.Level.INFO);

  PreShooterSubsystem preShooterSubsystem = RobotContainer.preShooterSubsystem;
  IntakeSubsystem intakeSubsystem = RobotContainer.intakeSubsystem;
  VisionSubsystem visionSubsystem = RobotContainer.visionSubsystem;

  Timer preshooterTimer = new Timer();
  boolean weAreDone = false;

  ShooterDecider.PewPewData pewPewData = new ShooterDecider.PewPewData();

  public PullTheTriggerForOneCommand() {
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(preshooterTimer.get() < 0.6) {    // .7 to .5 to .4
        preShooterSubsystem.preshooterOn(1);
        intakeSubsystem.overrideIntakeBeltForShooting(0.0);
        //intakeSubsystem.overrideIntakeWheelBarForShooting(0.0);
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
