// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;

import java.util.HashMap;
import java.util.Map;


public abstract class GetReadyToShootCommand extends CommandBase {
  Logger logger = EventLogging.getLogger(getClass(), EventLogging.Level.INFO);

  /** Creates a new GetReadyToShootCommand. */
  ShooterSubsystem shooterSubsystem;
  TurretSubsystem turretSubsystem;
  RumbleCommand rumbleCommandOperator, rumbleCommandDriver;

  Map<String, Object> pewPewData = new HashMap<>();
  Gson gson = new Gson();

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

    pewPewData.put("shooter.speed.ratio", terror);
    pewPewData.put("shooter.speed.requested", ts);
    pewPewData.put("shooter.speed.actual", ta);

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

    pewPewData.put("hood.position.ratio", herror);
    pewPewData.put("hood.position.requested", hs);
    pewPewData.put("hood.position.actual", ha);

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

    pewPewData.put("turret.position.error", trerror);
    pewPewData.put("turret.position.requested", trs);
    pewPewData.put("turret.position.actual", tra);

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
    pewPewData.clear();

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
    return answer;
  }

  void logPewPewData() {
    String json = gson.toJson(pewPewData);
    logger.info("pewpew! {}", json);
  }

  void showReady(boolean ready) {
    SmartDashboard.putBoolean("shooter.ready", ready);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted){
      logPewPewData();
      rumbleCommandOperator.schedule();
      rumbleCommandDriver.schedule();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean ready = everythingIsReady();
    showReady(ready);
    return ready;
  }
}