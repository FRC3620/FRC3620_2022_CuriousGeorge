package frc.robot.commands;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DougNavigationTest extends SequentialCommandGroup {

  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

  public DougNavigationTest(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, TurretSubsystem turretSubsystem) {

    addCommands(
      new LogCommand("starting " + getClass().getName()),

      new setInitialNavXOffsetCommand(driveSubsystem, 90),
      new ResetNavXPositionCommand(),

      new LogCommand("done setting NavXOffset"),

      new AutoDriveCommand(40, 90, .5, 90, driveSubsystem),

      new LogCommand(getClass().getName() + " done")
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
  }

}