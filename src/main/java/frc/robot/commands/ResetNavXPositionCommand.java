package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class ResetNavXPositionCommand extends InstantCommand {
  public ResetNavXPositionCommand() {
    super();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called once when the command executes
  @Override
  public void initialize() {
    RobotContainer.driveSubsystem.resetNavXDisplacement();
  }
}
