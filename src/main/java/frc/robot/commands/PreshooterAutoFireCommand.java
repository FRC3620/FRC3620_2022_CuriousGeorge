// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PreShooterSubsystem;


public class PreshooterAutoFireCommand extends CommandBase {
  /** Creates a new PreshooterAutoFireCommand. */
  PreShooterSubsystem preShooterSubsystem = RobotContainer.preShooterSubsystem;
  IntakeSubsystem intakeSubsystem = RobotContainer.intakeSubsystem;

  boolean intakeRunning = false;
  Timer preshooterTimer = new Timer();


  public PreshooterAutoFireCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem, preShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    preshooterTimer.reset();
    preshooterTimer.start();
    if(intakeSubsystem.getIntakeSpeed() > 0) {
      intakeRunning = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.spinIntakeBelt(0.0);
    spinPreShooter();
  }


  public void spinPreShooter(){
    preShooterSubsystem.preshooterOn(1.0);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    preShooterSubsystem.preshooterOff();
    intakeSubsystem.spinIntakeBelt(0.4);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(preshooterTimer.get() > 1){
      return true;
    }
    return false;
  }
}
