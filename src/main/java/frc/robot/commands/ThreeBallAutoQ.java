package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;

public class ThreeBallAutoQ extends SequentialCommandGroup {
  public ThreeBallAutoQ(DriveSubsystem driveSubsystem) {

      addCommands(
      new setInitialNavXOffsetCommand(driveSubsystem,242)
      ,
      new AutoDriveCommand(79, 195, .4, 215, driveSubsystem)
      //Picks up the ball in front of it and shoots 2 balls 
      ,
      new WaitCommand(.5)
      ,
      //picks up ball
      new AutoDriveCommand(245, 130, .3, 135, driveSubsystem 
      ));
      
  }
}
