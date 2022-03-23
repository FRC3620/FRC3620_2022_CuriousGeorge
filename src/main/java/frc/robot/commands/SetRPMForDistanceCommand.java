package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.miscellaneous.ShooterCalculator;

public class SetRPMForDistanceCommand extends InstantCommand {
  double shooterDistance;
  double mainRPM;
  public SetRPMForDistanceCommand(double distance) { //distance is feet away from target
    // Use addRequirements() here to declare subsystem dependencies.
  shooterDistance = distance;
  mainRPM = ShooterCalculator.calcMainRPM(shooterDistance);
  }

  @Override
  public void initialize() {
    RobotContainer.shooterSubsystem.setMainRPM(mainRPM);
    RobotContainer.shooterSubsystem.setBackRPM(ShooterCalculator.calculateBackspinRPM(mainRPM));
  }
}