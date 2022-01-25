package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;

public class FourBallAutoQ extends SequentialCommandGroup {
  public FourBallAutoQ(DriveSubsystem driveSubsystem) {

      addCommands(
      new setInitialNavXOffsetCommand(driveSubsystem, 145)
      ,
      new AutoDriveCommand(51, 145, .3, 150, driveSubsystem) //drive to position A
      ,
      new WaitCommand(.5) //shooting 2 balls
      ,
      new AutoDriveCommand(100, 200, .3, 133, driveSubsystem)
      ,
      new WaitCommand(.1)
      ,
      new AutoDriveCommand(105, 135, 0.3, 130, driveSubsystem) 
      ,
      new AutoDriveCommand(220, 325, .3, 325, driveSubsystem)
      ,
      new WaitCommand(.5) //shooting
      );
  }
}
