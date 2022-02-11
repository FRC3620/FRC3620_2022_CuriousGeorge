// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberTiltTestCommandIn extends CommandBase {
  ClimberSubsystem climberSubsystem = RobotContainer.climberSubsystem;
  protected Timer m_timer = new Timer();
  
  
  /** Creates a new ClimberTiltTestCommandOut. */
  public ClimberTiltTestCommandIn() {
    addRequirements(climberSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climberSubsystem.climberArmTiltIn();
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    climberSubsystem.climberArmTiltOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(5);
    
  }
}
