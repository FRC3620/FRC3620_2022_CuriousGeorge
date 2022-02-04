
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class TestAuto extends SequentialCommandGroup {
  public TestAuto(DriveSubsystem driveSubsystem) {
      addCommands(
      new setInitialNavXOffsetCommand(driveSubsystem, 90)
      ,
      new AutoDriveCommand(5*12, -90, .3, 180, driveSubsystem),
      new AutoSpinCommand(0.4, 180, driveSubsystem),
      new AutoDriveCommand(5*12, 180, .3, -90, driveSubsystem),
      new AutoSpinCommand(0.4, -90, driveSubsystem),
      new AutoDriveCommand(5*12, 90, .3, 0, driveSubsystem),
      new AutoSpinCommand(0.4, 0, driveSubsystem),
      new AutoDriveCommand(5*12, 0, .3, 90, driveSubsystem),
      new AutoSpinCommand(0.4, 90, driveSubsystem)
      //new AutoDriveCommand(4*12, 270, .2, 180, driveSubsystem),
      );
  }
}