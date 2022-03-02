// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.miscellaneous.CANSparkMaxSendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.RobotContainer;

public class HoodDownCommand extends CommandBase {

  private ShooterSubsystem shooterSubsystem;

  CANSparkMaxSendable hoodMotor = RobotContainer.shooterSubsystemHoodMax;

  double previousHoodPosition = 0;
  double currentHoodPosition = 0;
  Timer hoodTimer = new Timer();

  /** Creates a new HoodDownCommand. */
  public HoodDownCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hoodTimer.reset();
    hoodTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setHoodPower(0.1);

    if (hoodTimer.get() > 0.1){
      previousHoodPosition = currentHoodPosition;
      hoodTimer.reset();
    }

    if (hoodMotor != null){
      currentHoodPosition = shooterSubsystem.getHoodPosition();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setHoodPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((currentHoodPosition - previousHoodPosition) <= 2 ){
      shooterSubsystem.resetHoodEncoder();
      return true;
    }
    return false;
  }
}
