package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class SetHoodPositionCommand extends InstantCommand {
  public SetHoodPositionCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {
    RobotContainer.shooterSubsystem.setHoodPositionToHome();
  }
}