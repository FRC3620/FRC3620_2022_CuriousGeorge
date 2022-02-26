// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.ShootingDataLogger;
import frc.robot.miscellaneous.ShooterCalculator;
import frc.robot.subsystems.ShooterSubsystem;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.IFastDataLogger;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ShooterTestCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_subsystem;

  public final static Logger logger = EventLogging.getLogger(ShooterTestCommand.class, Level.INFO);

  IFastDataLogger dataLogger;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShooterTestCommand(ShooterSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    SmartDashboard.putNumber("top.set", 0.0);
    SmartDashboard.putNumber("back.set", 0.0);
    SmartDashboard.putBoolean("manual backspin", false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    boolean shouldDoDataLogging = SmartDashboard.getBoolean("datalogging.enabled", false);
    if (shouldDoDataLogging) {
      dataLogger = ShootingDataLogger.getShootingDataLogger("shooter_m", m_subsystem);
      dataLogger.start();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double t = SmartDashboard.getNumber("top.set", 0.0);
    double b = SmartDashboard.getNumber("back.set", 0.0);

    //logger.info ("execute: {} {}", t, b);
    m_subsystem.setMainRPM(t);
    double backspinRPM = ShooterCalculator.calculateBackspinRPM(t);
    SmartDashboard.putNumber("back.Calculated", backspinRPM);

    if( SmartDashboard.getBoolean ("manual backspin", true)) {
        m_subsystem.setBackRPM(b);

    }
    else{ 
         m_subsystem.setBackRPM(backspinRPM);
    }
  
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setMainRPM(0);
    m_subsystem.setBackRPM(0);
    if (dataLogger != null) {
      // dataLogger.done();
      dataLogger = null;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}