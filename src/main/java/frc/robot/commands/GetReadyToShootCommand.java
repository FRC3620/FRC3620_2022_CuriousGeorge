// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public abstract class GetReadyToShootCommand extends CommandBase {
  /** Creates a new GetReadyToShootCommand. */
  ShooterSubsystem shooterSubsystem;
  TurretSubsystem turretSubsystem;
  RumbleCommand rumbleCommandOperator, rumbleCommandDriver;
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
    SmartDashboard.putBoolean("shooter.ready.shooter", false);
    SmartDashboard.putBoolean("shooter.ready.hood", false);
    SmartDashboard.putBoolean("shooter.ready.turret", false);
    SmartDashboard.putBoolean("shooter.ready", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  public boolean isShooterUpToSpeed(){
    double ta = shooterSubsystem.getActualMainShooterVelocity();
    double ts = shooterSubsystem.getRequestedMainShooterVelocity();
    double terror = Double.NaN;
    if (ts != 0.0) {
      terror = ta / ts;
    }
    SmartDashboard.putNumber("shooter.error.shooter", terror);
    boolean answer;
    if (terror >= 0.98 && terror <= 1.02) {
      answer = true;
    }else {
      answer = false;
    }
    SmartDashboard.putBoolean("shooter.ready.shooter", answer);
    return answer;
  }

  public boolean isHoodInPosition(){
    double ha = shooterSubsystem.getHoodPosition();
    double hs = shooterSubsystem.getRequestedHoodPosition();
    double herror = Double.NaN;
    if (hs != 0.0) {
      herror = ha / hs;
    }
    SmartDashboard.putNumber("shooter.error.hood", herror);
    boolean answer;
    if (herror >= 0.98 && herror <= 1.02) {
      answer = true;
    }else {
      answer = false;
    }
    SmartDashboard.putBoolean("shooter.ready.hood", answer);
    return answer;
  }

  public boolean isTurretInPosition(){
    double tra = turretSubsystem.getCurrentTurretPosition();
    double trs = turretSubsystem.getRequestedTurretPosition();
    double trerror = Math.abs(trs - tra);
    SmartDashboard.putNumber("shooter.error.turret", trerror);
    boolean answer;
    if (trerror < 2) {
      answer = true;
    } else {
      answer = false;
    }
    SmartDashboard.putBoolean("shooter.ready.turret", answer);
    return answer;
  }

  public boolean everythingIsReady(){
    boolean answer = true;
    if (!isShooterUpToSpeed()){
      answer = false;
    }
    if (!isHoodInPosition()){
      answer = false;
    }
    if (!isTurretInPosition()){
      answer = false;
    }
    SmartDashboard.putBoolean("shooter.ready", answer);
    return answer;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted){
      rumbleCommandOperator.schedule();
      rumbleCommandDriver.schedule();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (everythingIsReady()){
      return true;
    }
    return false;
  }
}
