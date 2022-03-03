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

public class MassageOperatorToShootCommand extends CommandBase {
  VisionSubsystem visionSubsystem;
  RumbleCommand rumbleCommand;
  
  /** Creates a new FindTargetCommand. */
  public MassageOperatorToShootCommand(VisionSubsystem _vsubsystem, RumbleSubsystem _rsubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    visionSubsystem = _vsubsystem;

    rumbleCommand = new RumbleCommand(_rsubsystem, Hand.BOTH, 1.0, 0.5);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
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

    if (visionSubsystem.isTargetCentered()){
      return true;
    }

   return false;
  }
}