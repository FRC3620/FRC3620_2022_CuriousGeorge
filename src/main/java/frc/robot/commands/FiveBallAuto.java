package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class FiveBallAuto extends SequentialCommandGroup {
  public FiveBallAuto(DriveSubsystem driveSubsystem, VisionSubsystem VisionSubsystem) {

      addCommands(
      new setInitialNavXOffsetCommand(driveSubsystem, 90)
      ,
      new WaitCommand(.5) //shooting
      ,
      new AutoDriveCommand(40, 90, .3, 90, driveSubsystem)
      //,
      //new AutoDriveToCargoCommand(3*12, 90, .3, 90, driveSubsystem, VisionSubsystem) //drive to position A
      ,
      new WaitCommand(.5) //shooting
      ,
      new AutoDriveCommand(12, 180, .3, 205, driveSubsystem)
      ,
      new AutoDriveToCargoCommand(10*12, 200, .3, 225, driveSubsystem, VisionSubsystem) //drive to position B
      ,
      new WaitCommand(5) //shooting
      ,
      new AutoDriveCommand(11*12, 190, .3, 205, driveSubsystem)
      ,
      new AutoDriveCommand(10, 135, 0.3, 135, driveSubsystem) 
      ,
      new AutoDriveToCargoCommand(10*12, 135, .3, 135, driveSubsystem, VisionSubsystem)
      ,
      new WaitCommand(.5) //shooting
      //new AutoDriveCommand(4*12, 270, .2, 180, driveSubsystem),*/
      );
  }
}
