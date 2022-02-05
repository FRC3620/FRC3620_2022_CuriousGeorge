package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubSystem;

public class DriveToCargoTestAuto extends SequentialCommandGroup {
  public DriveToCargoTestAuto(DriveSubsystem driveSubsystem, VisionSubSystem visionSubsystem) {

      addCommands(
      new setInitialNavXOffsetCommand(driveSubsystem, 0)
      ,
      new AutoDriveToCargoCommand(11*12, 0, 0.3, 0, driveSubsystem, visionSubsystem)
      );
  }
}
