// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this projec
package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PreShooterSubsystem;
public class PullTheTriggerCommand extends CommandBase {
  PreShooterSubsystem preShooterSubsystem = RobotContainer.preShooterSubsystem;
  IntakeSubsystem intakeSubsystem = RobotContainer.intakeSubsystem;

  Timer preshooterTimer = new Timer();
  boolean weAreDone = false;
  public PullTheTriggerCommand() {
    // Use addRequirements() here to declare subsystem dependencies.

    // do not require the intake!!!!!!!!
    addRequirements(preShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    preshooterTimer.reset();
    preshooterTimer.start();
    weAreDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(preshooterTimer.get() < 1.0) {
        preShooterSubsystem.preshooterOn(1.0);
        intakeSubsystem.overrideIntakeBeltForShooting(0.0);
        intakeSubsystem.overrideIntakeWheelBarForShooting(0.0);
    } else if (preshooterTimer.get() < 2.0) {
        preShooterSubsystem.preshooterOff();
        intakeSubsystem.overrideIntakeBeltForShooting(0.4);
        intakeSubsystem.overrideIntakeWheelBarForShooting(0.0);
    } else {
        weAreDone = true;
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      //intakeSubsystem.startPreviousCommand(this);
    }
    intakeSubsystem.clearIntakeShootingOverrides();
    preShooterSubsystem.preshooterOff();
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(weAreDone){
      return true;
    }
    return false;
  }
}