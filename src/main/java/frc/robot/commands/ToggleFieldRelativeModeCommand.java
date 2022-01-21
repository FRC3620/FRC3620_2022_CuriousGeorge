package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class ToggleFieldRelativeModeCommand extends InstantCommand {
  DriveSubsystem m_DriveSubsystem;
  public ToggleFieldRelativeModeCommand(DriveSubsystem driveSubsystem) {
    m_DriveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DriveSubsystem.switchFieldRelative();
  }
}
