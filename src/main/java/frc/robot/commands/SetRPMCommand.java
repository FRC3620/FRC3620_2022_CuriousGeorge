package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.miscellaneous.ShooterCalculator;

public class SetRPMCommand extends InstantCommand {
  double shooterDistance;
  public SetRPMCommand(double distance) { //distance is feet away from target
    // Use addRequirements() here to declare subsystem dependencies.
  shooterDistance = distance;
  }

  @Override
  public void initialize() {
    RobotContainer.shooterSubsystem.setMainRPM(ShooterCalculator.calcMainRPM(shooterDistance));
    RobotContainer.shooterSubsystem.setBackRPM(ShooterCalculator.calculateBackspinRPM(shooterDistance));
  }
}