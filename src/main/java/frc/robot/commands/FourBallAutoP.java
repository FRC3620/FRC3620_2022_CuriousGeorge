package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class FourBallAutoP extends SequentialCommandGroup {
  public FourBallAutoP(DriveSubsystem driveSubsystem, VisionSubsystem VisionSubsystem) {

      addCommands(
      new setInitialNavXOffsetCommand(driveSubsystem, 90)
      ,
      new WaitCommand(.5) //shooting
      ,
      new AutoDriveCommand(40, 90, .3, 90, driveSubsystem) //drive to position A
      ,
      new AutoDriveCommand(21.5*12, 183, .3, 135, driveSubsystem) //drive to position D
      ,
      new AutoDriveToCargoCommand(2*12, 135, .3, 135, driveSubsystem, VisionSubsystem)
      ,
      new WaitCommand(.5) //shooting
      //new AutoDriveCommand(4*12, 270, .2, 180, driveSubsystem),*/
      );
  }
}
