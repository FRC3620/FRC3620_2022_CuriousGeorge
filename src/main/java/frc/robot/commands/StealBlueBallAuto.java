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

public class StealBlueBallAuto extends SequentialCommandGroup {
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);
  
  public StealBlueBallAuto(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, TurretSubsystem turretSubsystem, IntakeSubsystem intakeSubsystem){
    addCommands(
      new StartShooterDataLoggingCommand(getClass().getSimpleName(), 20.0),

      new setInitialNavXOffsetCommand(driveSubsystem, 180),
  
      new MoveTurretCommand(turretSubsystem, 180), 

      new SetHoodAngleForDistanceCommand(8),

      new SetRPMForDistanceCommand(8),
  
      new IntakeArmDownCommand(), 

      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new AutoDriveCommand(54, 180, .5, 180, driveSubsystem)
        ), 
        new GetVisionReadyToShootCommand(),
        new IntakeOnCommand()
      ),

      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new AutoShootCommand(),
          new PullTheTriggerCommand()
        ),
        new IntakeOffCommand(intakeSubsystem)
      ),

      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new AutoSpinCommand(-0.5, 85, driveSubsystem),
          new AutoDriveCommand(16, 85, 0.5, 85, driveSubsystem),
          new WaitCommand(1)
        ),
        new IntakeOnCommand()
      ),

      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new AutoDriveCommand(88, 180, 0.5, 85, driveSubsystem)
        ),
        new AutoBloopCommand(),
        new PushBallUpCommand(),
        new IntakeArmUpCommand()
      ),

       new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new SetRPMForDistanceCommand(1),
          new WaitCommand(2),
          new PullTheTriggerCommand()
        ),
        new IntakeOffCommand(intakeSubsystem)
      ),
        
      new ShooterOffCommand(),

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
