package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;

public class FourBallAutoP extends SequentialCommandGroup {
  public FourBallAutoP(DriveSubsystem driveSubsystem) {

      addCommands(
      new setInitialNavXOffsetCommand(driveSubsystem, 90)
      ,
      new WaitCommand(.5) //shooting
      ,
      new AutoDriveCommand(42, 90, .3, 90, driveSubsystem) //drive to position A
      ,
      new AutoDriveCommand(113, 195, .3, 205, driveSubsystem) //drive to position B
      ,
      new WaitCommand(.5) //shooting
      ,
      new AutoDriveCommand(130, 200, .3, 135, driveSubsystem)
      ,
      new AutoDriveCommand(58, 135, 0.3, 135, driveSubsystem) 
      ,
      new WaitCommand(.5) //shooting
      //new AutoDriveCommand(4*12, 270, .2, 180, driveSubsystem),*/
      );
  }
}
