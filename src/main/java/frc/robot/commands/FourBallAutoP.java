package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class FourBallAutoP extends SequentialCommandGroup {
  public FourBallAutoP(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {

      addCommands(
      new setInitialNavXOffsetCommand(driveSubsystem, 90)
      ,
      new WaitCommand(.5) //shooting
      ,
      new AutoDriveCommand(40, 90, .5, 90, driveSubsystem) //drive to position A
      ,
      new AutoDriveCommand(12*12, 180, 1, 135, driveSubsystem) //drive to position D
      ,
      new AutoDriveCommand(108, 205, .5, 135, driveSubsystem)
      ,
      new AutoDriveToCargoCommand(10, 135, .5, 135, driveSubsystem, visionSubsystem)
      ,
      new WaitCommand(.5) //shooting
      //new AutoDriveCommand(4*12, 270, .2, 180, driveSubsystem),*/
      );
  }
}
