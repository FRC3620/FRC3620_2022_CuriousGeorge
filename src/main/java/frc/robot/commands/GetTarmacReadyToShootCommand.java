// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.miscellaneous.ShooterCalculator;

public class GetTarmacReadyToShootCommand extends GetReadyToShootCommand {
  /** Creates a new GetTarmacReadyToShootCommand. */
  public GetTarmacReadyToShootCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();

    double distance = 10;
    double main_rpm = ShooterCalculator.calcMainRPM(distance);
    shooterSubsystem.setMainRPM(main_rpm);
    shooterSubsystem.setBackRPM(ShooterCalculator.calculateBackspinRPM(main_rpm));
    shooterSubsystem.setHoodPositionToDegrees(ShooterCalculator.calcHoodPosition(distance));

    turretSubsystem.setTurretPosition(180);
  }
}
