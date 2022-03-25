package frc.robot.commands;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.miscellaneous.ShooterCalculator;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class FiveBallAutoP extends SequentialCommandGroup {
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);
  
  public FiveBallAutoP(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, TurretSubsystem turretSubsystem, IntakeSubsystem intakeSubsystem){
    addCommands(
      new StartShooterDataLoggingCommand(getClass().getSimpleName(), 20.0),

      new setInitialNavXOffsetCommand(driveSubsystem, 90),

      new MoveTurretCommand(turretSubsystem, 180),

      new SetHoodAngleForDistanceCommand(8),

      new SetRPMForDistanceCommand(8),
  
      new IntakeArmDownCommand(), 

      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new AutoDriveCommand(40, 90, .6, 90, driveSubsystem)
        ), 
        new IntakeOnCommand()
      ),

      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new AutoShootCommand(),
          new PullTheTriggerCommand()
        ),
        new IntakeOffCommand(intakeSubsystem)
      ),

      new LogCommand("Done with first shots"),

      new MoveTurretCommand(turretSubsystem, 190),

      new SetHoodAngleForDistanceCommand(23),

      new SetRPMForDistanceCommand(23),
        
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new IntakeArmDownCommand(),
          new AutoDriveCommand(130, 200, 0.8, 205, driveSubsystem)
          //new AutoDriveToCargoCommand(120, 200, 0.5, 225, driveSubsystem, visionSubsystem)
        ),
        new IntakeOnCommand()
      ),

      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new AutoDriveCommand(145, 165, 0.7, 145, driveSubsystem)
        ),
        new IntakeOnCommand(),
        new LogCommand("intake works")
      ),

      new AutoShootCommand(),
      new LogCommand("found target"),
      new ParallelCommandGroup(
          new IntakeOnCommand(),
          new GetVisionReadyToShootCommand(),
          new SequentialCommandGroup(
              new PullTheTriggerForOneCommand(),
              new LogCommand("shot once"),
              new PushBallUpCommand(),
              new LogCommand("ball is ready to shoot"),
              new PullTheTriggerForOneCommand(),
              new PushBallUpCommand(),
              new PullTheTriggerCommand()
          )
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
