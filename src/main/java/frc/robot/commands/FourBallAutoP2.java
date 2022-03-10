package frc.robot.commands;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class FourBallAutoP2 extends SequentialCommandGroup {
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);
  
  public FourBallAutoP2(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, TurretSubsystem turretSubsystem, IntakeSubsystem intakeSubsystem){
    addCommands(
      new setInitialNavXOffsetCommand(driveSubsystem, 90),
  
      new MoveTurretCommand(turretSubsystem, 180), 
  
      new IntakeArmDownCommand(), 

      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new AutoDriveCommand(39.5, 90, .8, 90, driveSubsystem)
        ), 
        new IntakeOnCommand()
      ),

      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new AutoShootCommand(),
          new PullTheTriggerCommand(),
          new PullTheTriggerCommand()
        ),
        new IntakeOffCommand(intakeSubsystem)
      ),

      new LogCommand("Done with first shots"),
        
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new IntakeArmDownCommand(),
          new AutoDriveCommand(130, 200, 0.8, 205, driveSubsystem),
          //new AutoDriveToCargoCommand(120, 200, 0.5, 225, driveSubsystem, visionSubsystem)
          new AutoDriveCommand(159, 175, 0.6, 170, driveSubsystem),
          new AutoDriveCommand(7*12, 315, 0.9, 135, driveSubsystem)
        ),
        new IntakeOnCommand()
      ),

      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new AutoShootCommand(),
          new PullTheTriggerCommand(),
          new PullTheTriggerCommand()
        ),
        new IntakeOffCommand(intakeSubsystem)
      ),

      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new IntakeArmUpCommand()
        ),
        new IntakeOffCommand(intakeSubsystem),
        new ShooterOffCommand()
      ),

      new LogCommand("All done")
    );  
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
