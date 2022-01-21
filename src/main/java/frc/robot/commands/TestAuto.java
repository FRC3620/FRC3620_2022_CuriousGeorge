
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class TestAuto extends SequentialCommandGroup {
  public TestAuto(DriveSubsystem driveSubsystem) {
      addCommands(
      //shoot
      new AutoDriveCommand(42, 180, .5, 180, driveSubsystem),
      new AutoDriveCommand(80, 60 , .5, 240, driveSubsystem),
      //shoot
      new AutoDriveCommand(120, 100, .3, 260, driveSubsystem)
      //shoot
      //new AutoDriveCommand(4*12, 270, .2, 180, driveSubsystem),
      );
  }
}