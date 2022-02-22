package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ThreeBallAutoQ extends SequentialCommandGroup {
  public ThreeBallAutoQ(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {

      addCommands(
      new setInitialNavXOffsetCommand(driveSubsystem, 153)
      //,
      //new AutoDriveCommand(20, 195, .4, 150, driveSubsystem)
      ,
      new AutoDriveToCargoCommand(4*12, 150, .3, 150, driveSubsystem, visionSubsystem)
      ,
      new WaitCommand(.5)       //Picks up the ball in front of it and shoots 2 balls 
      ,
      new AutoDriveCommand(130, 185, .3, 130, driveSubsystem  )
      ,
      new AutoDriveToCargoCommand(10*12, 135, .3, 135, driveSubsystem, visionSubsystem)
      );

      
  }
}
