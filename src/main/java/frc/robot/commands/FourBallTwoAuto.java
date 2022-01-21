package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class FourBallTwoAuto extends SequentialCommandGroup {
  //Start at position P(drawing on wall)
  public FourBallTwoAuto(DriveSubsystem driveSubsystem) {
      addCommands(
      //shoot
      new AutoDriveCommand(45, 185, .5, 180, driveSubsystem),
      //shoot
      new AutoDriveCommand(84, 185 , .5, 180, driveSubsystem)
      //shootx2
      );
  }
}
