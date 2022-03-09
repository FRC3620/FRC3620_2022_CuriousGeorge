package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ThreeBallAutoQ extends SequentialCommandGroup {
  public ThreeBallAutoQ(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, TurretSubsystem turretSubsystem) {

      addCommands(
      new setInitialNavXOffsetCommand(driveSubsystem, 145)
      ,
      new ScheduleCommand(new MoveTurretCommand(turretSubsystem, 180))
      ,
      new ScheduleCommand(new IntakeArmDownCommand())
      ,
      new ScheduleCommand(new IntakeSpinCommand())
      ,
      new AutoDriveToCargoCommand(4*12, 150, .3, 150, driveSubsystem, visionSubsystem)
      ,
      new AutoShootCommand()
      ,
      new AutoDriveCommand(145, 185, .3, 135, driveSubsystem)
      ,
      new AutoDriveToCargoCommand(10*12, 135, .3, 135, driveSubsystem, visionSubsystem)
      ,
      new AutoShootCommand()
      );

      
  }
}
