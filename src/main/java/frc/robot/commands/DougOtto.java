package frc.robot.commands;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DougOtto extends SequentialCommandGroup {
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

  public DougOtto(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, TurretSubsystem turretSubsystem) {

    addCommands(
      new LogCommand("starting " + getClass().getName()),

      new setInitialNavXOffsetCommand(driveSubsystem, 90),

      new LogCommand("done setting NavXOffset"),

      new WaitCommand(3.0), // wait for turret to home

      new MoveTurretCommand(turretSubsystem, 180), 

      new LogCommand("done with MoveTurret"),

      new IntakeArmDownCommand(), 
      
      new LogCommand("done with IntakeArmDownCommand"),

      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new LogCommand("starting IntakeSpin"),
          new IntakeOnCommand()
        ),
        new SequentialCommandGroup(
          new LogCommand("starting AutoDrive"),

          new AutoDriveCommand(40, 90, .5, 90, driveSubsystem),

          new LogCommand("finishing AutoDrive"),

          new WaitCommand(3),

          new LogCommand("finishing Wait"),

          new AutoShootCommand(),

          new LogCommand("pulling the trigger"),

          new PullTheTriggerCommand(),

          new LogCommand(getClass().getName() + " interior SequentialCommandGroup done")
        )
      )
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

    logger.info ("dougotto requirements: {}", getRequirements());
  }

  class LogCommand extends InstantCommand {
    String m;
    Object o;
    LogCommand(String message) {
      this.m = message;
      this.o = null;
    }

    LogCommand(String message, Object[] args) {
      this.m = message;
      this.o = args;
    }

    @Override
    public void initialize() {
      if (o == null) {
        logger.info(m);
      } else {
        logger.info(m, o);
      }
    }
  
    @Override
    public boolean runsWhenDisabled() {
      return true;
    }
  }
}
