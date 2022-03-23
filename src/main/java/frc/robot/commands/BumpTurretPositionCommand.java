package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.TurretSubsystem;

public class BumpTurretPositionCommand extends InstantCommand {
  private final TurretSubsystem turretSubsystem;

  public BumpTurretPositionCommand(TurretSubsystem turretSubsystem) {
    this.turretSubsystem = turretSubsystem;
    addRequirements(this.turretSubsystem);

    SmartDashboard.putNumber("turret.bump_increment", 1.0);
  }

  @Override
  public void initialize() {
    double bump = SmartDashboard.getNumber("turret.bump_increment", 1.0);
    double newTurretPosition = turretSubsystem.getRequestedTurretPosition() + bump;
    turretSubsystem.setTurretPosition(newTurretPosition);
  }
}
