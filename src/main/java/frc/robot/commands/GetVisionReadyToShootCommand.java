// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.miscellaneous.ShooterCalculator;
import frc.robot.subsystems.VisionSubsystem;

public class GetVisionReadyToShootCommand extends GetReadyToShootCommand {
  /** Creates a new GetVisionReadyToShootCommand. */
  VisionSubsystem visionSubsystem;
  Timer turretTimer = new Timer();
  public GetVisionReadyToShootCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    visionSubsystem = RobotContainer.visionSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    turretTimer.reset();
    turretTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
    if (turretTimer.get() > 0.3){
      //seconds
      setUpStuffToShoot();
      turretTimer.reset(); 
    }
  }

  public void setUpStuffToShoot(){
    double currentTurretPosition = turretSubsystem.getCurrentTurretPosition();
    if (visionSubsystem.isTargetFound()){
      double spinTurretDegrees = visionSubsystem.getTargetXDegrees();
      double targetx = spinTurretDegrees + currentTurretPosition;
      turretSubsystem.setTurretPosition(targetx);

      //double targetDistance = ShooterCalculator.calcDistanceFromHub(visionSubsystem.getTargetYLocation());
      //double targetRPM = ShooterCalculator.calcMainRPM(targetDistance);

      //shooterSubsystem.setMainRPM(targetRPM);
      //shooterSubsystem.setBackRPM(ShooterCalculator.calculateBackspinRPM(targetRPM));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
