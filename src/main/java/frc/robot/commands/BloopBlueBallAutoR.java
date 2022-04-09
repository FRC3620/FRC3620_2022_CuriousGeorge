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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class BloopBlueBallAutoR extends SequentialCommandGroup {
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);
  
  public BloopBlueBallAutoR(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, TurretSubsystem turretSubsystem, IntakeSubsystem intakeSubsystem){
    addCommands(
      new StartShooterDataLoggingCommand(getClass().getSimpleName(), 20.0),

      new setInitialNavXOffsetCommand(driveSubsystem, 238),
  
      new MoveTurretCommand(turretSubsystem, 180), 

      new LogCommand("Moved turret"),
  
      new IntakeArmDownCommand(), 

      new XParallelDeadlineGroup(
        new XSequentialCommandGroup(
          new LogCommand("Starting AutoDrive"),
          new AutoDriveCommand(55, 238, .5, 238, driveSubsystem)
        ), 
        new GetVisionReadyToShootCommand(),
        new IntakeOnCommand()
      ),

      new LogCommand("Done with AutoDrive"),

      new XParallelDeadlineGroup(
        new XSequentialCommandGroup(
          new AutoShootCommand(),
          new PullTheTriggerCommand()
        ),
        new IntakeOffCommand(intakeSubsystem)
      ),

      new LogCommand("Done with first shots"),
        
      new XParallelDeadlineGroup(
        new XSequentialCommandGroup(
          new AutoSpinCommand(0.5, 0, driveSubsystem),
          new AutoDriveCommand(34, 270, 0.5, 0, driveSubsystem),
          new AutoDriveCommand(50, 0, 0.5, 0, driveSubsystem),
          new AutoDriveCommand(50, 180, 0.5, 0, driveSubsystem)
        ),
        new AutoBloopCommand(),
        new IntakeOnCommand()
      ),

      new XParallelDeadlineGroup(
        new XSequentialCommandGroup(
          new PullTheTriggerCommand()
        ),
        new IntakeOffCommand(intakeSubsystem)
      ),

      new ShooterOffCommand(),

      new LogCommand("All done")
    );
    logger.info ("requirements for {}: {}", toString(), getRequirements());
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
