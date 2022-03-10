// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class GetReadyToClimbCommand extends CommandBase {
  /** Creates a new GetReadyToClimb. */
  TurretSubsystem turretSubsystem;
  IntakeArmSubsystem intakeArmSubsystem;
  ShooterSubsystem shooterSubsystem;

  public GetReadyToClimbCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    turretSubsystem = RobotContainer.turretSubsystem;
    intakeArmSubsystem = RobotContainer.intakeArmSubsystem;
    shooterSubsystem = RobotContainer.shooterSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turretSubsystem.setTurretPosition(0);
    intakeArmSubsystem.retractIntakeArm();
    shooterSubsystem.setHoodPositionToHome();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
