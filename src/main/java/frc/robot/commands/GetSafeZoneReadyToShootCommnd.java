// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class GetSafeZoneReadyToShootCommnd extends GetReadyToShootCommand {
  /** Creates a new GetSafeZoneReadyToShootCommnd. */
  public GetSafeZoneReadyToShootCommnd() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    shooterSubsystem.setMainRPM(1920);
    shooterSubsystem.setBackRPM(1760);
    shooterSubsystem.setHoodPositionToDegrees(57.5);
    turretSubsystem.setTurretPosition(180);
  }
}
