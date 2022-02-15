// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FindTargetCommand extends CommandBase {
  TurretSubsystem turretSubsystem;
  VisionSubsystem VisionSubsystem;
  Timer turretTimer = new Timer();
  
  /** Creates a new FindTargetCommand. */
  public FindTargetCommand(TurretSubsystem _subsystem, VisionSubsystem _vsubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_subsystem);
    turretSubsystem = _subsystem;
    VisionSubsystem = _vsubsystem;
    SmartDashboard.putBoolean("Target Found", true);
     
   
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

   double spinTurretDegrees = 0;
    double positionInFrame = VisionSubsystem.getTargetXLocation();
    SmartDashboard.putNumber("target location",positionInFrame);
    double currentTurretPosition = turretSubsystem.getCurrentTurretPosition();
    if (positionInFrame>=0){
       
      spinTurretDegrees = ((positionInFrame - 0.5)/0.0825)*5;
       double  Targetx = spinTurretDegrees + currentTurretPosition;
       turretSubsystem.setTurretPosition(Targetx);
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

