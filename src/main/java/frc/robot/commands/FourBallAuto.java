
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class FourBallAuto extends SequentialCommandGroup {
  /**
   * Creates a new SimpleAutoCommand.
   */
  //Start at position P(drawing on wall)
  public FourBallAuto(DriveSubsystem driveSubsystem) {
      addCommands(
      //shoot
      new AutoDriveCommand(45, 180, .5, 180, driveSubsystem),
      //shoot
      new AutoDriveCommand(300, 80 , .5, 180, driveSubsystem)
      //shootx2
      );
  }
}
