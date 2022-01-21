package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;

public class DougTestAutoDrive extends SequentialCommandGroup {
  public DougTestAutoDrive(DriveSubsystem driveSubsystem) {
    addCommands(
      new AutoDriveBeingDevelopedCommand(60, 0, .2, 0, driveSubsystem),
      new WaitCommand(2.0),
      new AutoDriveBeingDevelopedCommand(12, 90, .2, 0, driveSubsystem),
      new WaitCommand(2.0),
      new AutoDriveBeingDevelopedCommand(12, -90, .2, 0, driveSubsystem),
      new WaitCommand(2.0),
      new AutoDriveBeingDevelopedCommand(60, -180, .2, -90, driveSubsystem),
      new WaitCommand(2.0),

      new PrintCommand("all done!")
    );
  }
}