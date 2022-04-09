// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoBloopCommand extends GetReadyToShootCommand {
  /** Creates a new GetBloopReadyToShootCommand. */
  public AutoBloopCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();

    shooterSubsystem.setMainRPM(2000);
    shooterSubsystem.setBackRPM(366);
    shooterSubsystem.setHoodPositionToDegrees(50);

    turretSubsystem.setTurretPosition(180);
  }
}
