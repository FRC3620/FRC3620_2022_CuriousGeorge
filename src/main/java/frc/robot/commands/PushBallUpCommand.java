package frc.robot.commands;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this projec

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.ShooterDecider;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PreShooterSubsystem;
import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;

public class PushBallUpCommand extends CommandBase {
  Logger logger = EventLogging.getLogger(getClass(), EventLogging.Level.INFO);

  PreShooterSubsystem preShooterSubsystem = RobotContainer.preShooterSubsystem;
  IntakeSubsystem intakeSubsystem = RobotContainer.intakeSubsystem;

  Timer beltTimer = new Timer();
  boolean weAreDone = false;

  ShooterDecider.PewPewData pewPewData = new ShooterDecider.PewPewData();

  public PushBallUpCommand() {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(preShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    beltTimer.reset();
    beltTimer.start();
    weAreDone = false;

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
   intakeSubsystem.overrideIntakeBeltForShooting(0.4);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.clearIntakeShootingOverrides();
    preShooterSubsystem.preshooterOff();
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