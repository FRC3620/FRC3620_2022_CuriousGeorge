package frc.robot.commands;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

public class BlindAutoShootCommand extends GetVisionReadyToShootCommand {
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

  /** Creates a new AutoShootCommand. */
  public BlindAutoShootCommand(double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
  }


  /**
   * The end() method in GetReadyToShootVision will rumble the joysticks.
   * We don't want that, so we override.
   * @param interrupted
   */
  @Override
  public void end(boolean interrupted) {
    EventLogging.commandMessage(logger, interrupted);
    if (!interrupted) {
      logPewPewData();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished();
  }
}
