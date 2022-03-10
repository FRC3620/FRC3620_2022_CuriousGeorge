// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FindTargetCommand extends CommandBase {
  TurretSubsystem turretSubsystem;
  VisionSubsystem visionSubsystem;
  Timer turretTimer = new Timer();
  
  /** Creates a new FindTargetCommand. */
  public FindTargetCommand(TurretSubsystem _subsystem, VisionSubsystem _vsubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_subsystem);
    turretSubsystem = _subsystem;
    visionSubsystem = _vsubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    spinTurret();
    turretTimer.reset();
    turretTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (turretTimer.get() > 0.3){
      //seconds
      spinTurret();
      turretTimer.reset(); 
    }
  }

  public void spinTurret(){
    double currentTurretPosition = turretSubsystem.getCurrentTurretPosition();
    if (visionSubsystem.isTargetFound()){
      double spinTurretDegrees = visionSubsystem.getTargetXDegrees();
      double targetx = spinTurretDegrees + currentTurretPosition;
      turretSubsystem.setTurretPosition(targetx);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return false;
  }
}