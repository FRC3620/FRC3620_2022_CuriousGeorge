// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.ShooterDecider;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;

public abstract class GetReadyToShootCommand extends CommandBase {
  Logger logger = EventLogging.getLogger(getClass(), EventLogging.Level.INFO);

  ShooterSubsystem shooterSubsystem;
  TurretSubsystem turretSubsystem;
  RumbleCommand rumbleCommandOperator, rumbleCommandDriver;

  ShooterDecider.PewPewData pewPewData = new ShooterDecider.PewPewData();

  public GetReadyToShootCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    shooterSubsystem = RobotContainer.shooterSubsystem;
    turretSubsystem = RobotContainer.turretSubsystem;
    rumbleCommandOperator = new RumbleCommand(RobotContainer.operatorRumbleSubsystem);
    rumbleCommandDriver = new RumbleCommand(RobotContainer.driverRumbleSubsystem);
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ShooterDecider.showNotReady();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  public boolean everythingIsReady(){
    pewPewData.clear();

    boolean answer = true;
    if (!ShooterDecider.isShooterUpToSpeed(pewPewData)){
      answer = false;
    }
    if (!ShooterDecider.isHoodInPosition(pewPewData)){
      answer = false;
    }
    if (!ShooterDecider.isTurretInPosition(pewPewData)){
      answer = false;
    }
    return answer;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted){
      logPewPewData();
      if (DriverStation.isTeleopEnabled()){
        rumbleCommandOperator.schedule();
        rumbleCommandDriver.schedule();
      }
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean ready = everythingIsReady();
    ShooterDecider.showReady(ready);
    return ready;
  }

  void logPewPewData() {
    ShooterDecider.logPewPewData(logger, "ready to", pewPewData);
  }
}