package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class FourBallAutoQ extends SequentialCommandGroup {
  public FourBallAutoQ(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {

      addCommands(
      new setInitialNavXOffsetCommand(driveSubsystem, 145)
      ,
      new AutoDriveToCargoCommand(4*12, 145, .3, 145, driveSubsystem, visionSubsystem)
      ,
      new WaitCommand(.5) //shoot 2 balls
      ,
      new AutoDriveCommand(130, 185, .3, 133, driveSubsystem)
      ,
      new WaitCommand(.1)
      ,
      new AutoDriveCommand(30, 135, 0.3, 130, driveSubsystem) 
      ,
      new AutoDriveToCargoCommand(100, 135, .3, 135, driveSubsystem, visionSubsystem)
      ,
     // new AutoDriveCommand(220, 325, .3, 325, driveSubsystem)
     // ,
      new WaitCommand(.5) //shooting
      );
  }
}
