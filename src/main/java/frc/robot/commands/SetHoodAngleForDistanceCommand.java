package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.miscellaneous.ShooterCalculator;

public class SetHoodAngleForDistanceCommand extends InstantCommand {
  double hoodDistance;
  public SetHoodAngleForDistanceCommand(double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
  hoodDistance = distance;
  }

  @Override
  public void initialize() {
    RobotContainer.shooterSubsystem.setHoodPositionToDegrees(ShooterCalculator.calcHoodAngle(hoodDistance));
  }
}