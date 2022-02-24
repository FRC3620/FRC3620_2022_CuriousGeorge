// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.RumbleSubsystem;

import frc.robot.subsystems.RumbleSubsystem.Hand;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MassageOperatorMethod extends CommandBase {
  TurretSubsystem turretSubsystem;
  VisionSubsystem visionSubsystem;
  RumbleCommand rumbleCommand;
  
  /** Creates a new FindTargetCommand. */
  public MassageOperatorMethod(TurretSubsystem _subsystem, VisionSubsystem _vsubsystem, RumbleSubsystem _rsubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_subsystem);
    turretSubsystem = _subsystem;
    visionSubsystem = _vsubsystem;

    rumbleCommand = new RumbleCommand(_rsubsystem, Hand.BOTH, 1.0, 0.5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Target Found", false);
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted){
      rumbleCommand.schedule();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    double positionInFrame = visionSubsystem.getTargetXLocation();

    if (positionInFrame > 0.4 && positionInFrame < 0.6){
      SmartDashboard.putBoolean("Target Found", true);
      return true;
    }

   return false;
  }
}