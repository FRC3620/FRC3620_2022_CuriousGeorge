package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class FiveBallAutoP extends SequentialCommandGroup {
  public FiveBallAutoP(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, TurretSubsystem turretSubsystem) {

      addCommands(
      new setInitialNavXOffsetCommand(driveSubsystem, 90)
      ,
      new ScheduleCommand(new MoveTurretCommand(turretSubsystem, 180))
      ,
      new ScheduleCommand(new IntakeArmDownCommand())
      ,
      new ScheduleCommand(new IntakeSpinCommand())
      ,
      new AutoDriveCommand(40, 90, .5, 90, driveSubsystem)
      //,
      //new AutoShootCommand()
      /*,
      new WaitCommand(3)
      ,
      new MoveTurretCommand(turretSubsystem, 90)
      ,
      new AutoDriveCommand(12, 200, .5, 205, driveSubsystem)
      ,
      new AutoDriveToCargoCommand(10*12, 200, .5, 225, driveSubsystem, visionSubsystem) //drive to position B
      ,
      new AutoShootCommand()
      ,
      new AutoDriveCommand(130, 190, .5, 135, driveSubsystem)
      ,
      new AutoDriveToCargoCommand(1*12, 135, .5, 135, driveSubsystem, visionSubsystem)
      ,
      new WaitCommand(2)
      ,
      new AutoShootCommand()
      */
      );
  }
}
