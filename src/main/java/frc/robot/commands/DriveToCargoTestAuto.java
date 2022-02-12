package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveToCargoTestAuto extends SequentialCommandGroup {
  public DriveToCargoTestAuto(DriveSubsystem driveSubsystem, VisionSubsystem VisionSubsystem) {

      addCommands(
      new setInitialNavXOffsetCommand(driveSubsystem, 225)
      ,
      new AutoDriveToCargoCommand(7*12, 215, 0.3, 215, driveSubsystem, VisionSubsystem)
      ,
      new AutoDriveCommand(5*12, 90, .3, 85, driveSubsystem)
      ,
      new AutoDriveToCargoCommand(10*12, 90, 0.3, 85, driveSubsystem, VisionSubsystem)
      ,
      new AutoDriveCommand(9*12, 210, 0.3, 145, driveSubsystem)
      ,
      new AutoDriveToCargoCommand(11*12, 135, 0.3, 135, driveSubsystem, VisionSubsystem)
      );
  }
}
