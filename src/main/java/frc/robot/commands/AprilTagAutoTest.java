package frc.robot.commands;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AprilTagVision;
import frc.robot.subsystems.DriveSubsystem;

public class AprilTagAutoTest extends SequentialCommandGroup {
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);
  
  public AprilTagAutoTest(DriveSubsystem driveSubsystem){
    addCommands(
      new SequentialCommandGroup(
        new LocateAprilTagCommand(driveSubsystem),
        new StrafeToAprilTagCommand(driveSubsystem),
        // strafes from april tag to cone stick 10in
        new AutoDriveCommand(22, 90, 0.3, 0, driveSubsystem)
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
