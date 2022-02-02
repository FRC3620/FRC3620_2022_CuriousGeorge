package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;

public class AutoTurnVisionLightOffCommand extends CommandBase {
  VisionSubsystem visionSubsystem;
  /**
   * Creates a new ToggleVisionLightCommand.
   */
  public AutoTurnVisionLightOffCommand(VisionSubsystem subsystem) {
    this.visionSubsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    visionSubsystem.turnVisionLightOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
