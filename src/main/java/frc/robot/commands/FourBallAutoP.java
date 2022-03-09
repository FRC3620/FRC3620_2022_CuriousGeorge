package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class FourBallAutoP extends SequentialCommandGroup {
  public FourBallAutoP(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, TurretSubsystem turretSubsystem) {

      addCommands(
      new setInitialNavXOffsetCommand(driveSubsystem, 90)
      ,
      new ScheduleCommand(new MoveTurretCommand(turretSubsystem, 180))
      ,
      new ScheduleCommand(new IntakeArmDownCommand())
      ,
      new ScheduleCommand(new IntakeSpinCommand())
      ,
      new AutoDriveCommand(40, 90, .5, 90, driveSubsystem) //drive to position A
      ,
      new AutoShootCommand()
      ,
      new AutoDriveCommand(12*12, 180, 1, 135, driveSubsystem) //drive to position D
      ,
      new AutoDriveCommand(108, 205, .5, 135, driveSubsystem)
      ,
      new AutoDriveToCargoCommand(10, 135, .5, 135, driveSubsystem, visionSubsystem)
      ,
      new WaitCommand(2)
      ,
      new AutoShootCommand()
      );
  }
}
