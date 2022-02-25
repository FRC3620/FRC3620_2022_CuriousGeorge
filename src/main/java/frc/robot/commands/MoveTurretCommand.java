/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TurretSubsystem;

public class MoveTurretCommand extends CommandBase {
  TurretSubsystem turretSubsystem;
  double desiredAngle;
  /**
   * 
   * Creates a new MoveTurretCommand.
   */
  public MoveTurretCommand(TurretSubsystem _subsystem, double _desiredAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_subsystem);
    turretSubsystem = _subsystem;
    desiredAngle = _desiredAngle;
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turretSubsystem.setTurretPosition(desiredAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    RobotContainer.visionSubsystem.turnVisionLightOn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}