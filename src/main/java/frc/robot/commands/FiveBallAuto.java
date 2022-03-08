package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class FiveBallAuto extends SequentialCommandGroup {
  public FiveBallAuto(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {

      addCommands(
      new setInitialNavXOffsetCommand(driveSubsystem, 90)
      ,
      new ScheduleCommand(new IntakeArmDownCommand())
      ,
      new WaitCommand(.5) //shooting
      ,
      new AutoDriveCommand(40, 90, .5, 90, driveSubsystem)
      //,
      //new AutoDriveToCargoCommand(3*12, 90, .3, 90, driveSubsystem, VisionSubsystem) //drive to position A
      ,
      new WaitCommand(.5) //shooting
      // ,
      //new AutoSpinCommand(1, 200, driveSubsystem)
      ,
      new AutoDriveCommand(12, 200, .5, 205, driveSubsystem)
      ,
      new AutoDriveToCargoCommand(10*12, 200, .5, 225, driveSubsystem, visionSubsystem) //drive to position B
      ,
      new WaitCommand(2) //shooting
      ,
      new AutoDriveCommand(130, 190, .5, 135, driveSubsystem)
      ,
      //new AutoDriveCommand(10, 135, .5, 135, driveSubsystem) 
      //,
      new AutoDriveToCargoCommand(1*12, 135, .5, 135, driveSubsystem, visionSubsystem)
      ,
      new WaitCommand(.5) //shooting
      //new AutoDriveCommand(4*12, 270, .2, 180, driveSubsystem),*/
      );
  }
}
