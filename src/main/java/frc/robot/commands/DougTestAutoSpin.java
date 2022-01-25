package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;

public class DougTestAutoSpin extends SequentialCommandGroup {
  public DougTestAutoSpin(DriveSubsystem driveSubsystem) {
    addCommands(
      new AutoSpinCommand(-0.4, 270, driveSubsystem),
      new WaitCommand(2.0),
      new AutoSpinCommand(0.4, 0, driveSubsystem),
      new WaitCommand(2.0),
      new AutoSpinCommand(0.4, 45, driveSubsystem),

      new PrintCommand("all done!")
    );
  }
}