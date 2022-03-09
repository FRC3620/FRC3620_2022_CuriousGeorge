package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class FourBallAutoR extends SequentialCommandGroup {
  public FourBallAutoR(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, TurretSubsystem turretSubsystem) {

      addCommands(
      new setInitialNavXOffsetCommand(driveSubsystem, 225)
      ,
      new ScheduleCommand(new MoveTurretCommand(turretSubsystem, 180))
      ,
      new ScheduleCommand(new IntakeArmDownCommand())
      ,
      new ScheduleCommand(new IntakeSpinCommand())
      ,
      new AutoDriveCommand(40, 225, .5, 225, driveSubsystem) //drive to position A
      ,
      new AutoShootCommand()
      ,
      new ScheduleCommand(new IntakeArmUpCommand())
      ,
      new AutoDriveCommand(12*12, 135, 0.5, 135, driveSubsystem) //drive to position D
      ,
      new ScheduleCommand(new IntakeArmDownCommand())
      ,
      new ScheduleCommand(new IntakeSpinCommand())
      ,
      new AutoDriveToCargoCommand(12*12, 135, 0.5, 135, driveSubsystem, visionSubsystem)
      ,
      new WaitCommand(2)
      ,
      new AutoShootCommand()
      );
  }
}
