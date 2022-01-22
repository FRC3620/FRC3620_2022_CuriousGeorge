
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class FourBallAuto extends SequentialCommandGroup {
  //Start at position P(drawing on wall)
  public FourBallAuto(DriveSubsystem driveSubsystem) {
      addCommands(
      //shoot
      new AutoDriveCommandDONOTUSE(45, 180, .5, 180, driveSubsystem),
      //shoot
      new AutoDriveCommandDONOTUSE(300, 80 , .5, 180, driveSubsystem)
      //shootx2
      );
  }
}
