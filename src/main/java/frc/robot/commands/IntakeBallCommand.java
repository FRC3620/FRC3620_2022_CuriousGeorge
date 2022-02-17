// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeBallCommand extends CommandBase {
  IntakeSubsystem intakeSubsystem = RobotContainer.intakeSubsystem;
  /** Creates a new IntakeBallCommand. */
  public IntakeBallCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeBall();
  }

  public void intakeBall() {
    intakeSubsystem.spinIntakeWheelBar(0.1);
    intakeSubsystem.spinIntakeBelt(0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.spinIntakeWheelBar(0.0);
    intakeSubsystem.spinIntakeBelt(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
