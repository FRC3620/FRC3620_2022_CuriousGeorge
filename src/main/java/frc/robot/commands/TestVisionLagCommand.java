package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;

public class TestVisionLagCommand extends CommandBase {
  Logger logger = EventLogging.getLogger(getClass(), EventLogging.Level.INFO);

  double t_lightOff, t_disappeared;
  boolean waiting_to_disappear, done;

  public TestVisionLagCommand() {
  }

  @Override
  public void initialize() {
    boolean seen = RobotContainer.visionSubsystem.isTargetFound();
    if (!seen) {
      logger.info("we never saw the target at the start");
      done = true;
    } else {
      t_lightOff = Timer.getFPGATimestamp();
      RobotContainer.visionSubsystem.turnRingLightOff();
      waiting_to_disappear = true;
      done = false;
    }
  }

  @Override
  public void execute() {
    boolean seen = RobotContainer.visionSubsystem.isTargetFound();
    double t = Timer.getFPGATimestamp();
    if (waiting_to_disappear) {
      if (!seen) {
        t_disappeared = t;
        RobotContainer.visionSubsystem.turnRingLightOn();
        waiting_to_disappear = false;
      }
    } else {
      if (seen) {
        double t_delta_disappear = t_disappeared - t_lightOff;
        double t_delta_appear = t - t_disappeared; // turned the light on at the same time the target disappeared
        logger.info("delay when disappearing={}, delay when appearing={}", t_delta_disappear, t_delta_appear);
        logger.info ("{} {} {}", t_lightOff, t_disappeared, t);
        SmartDashboard.putNumber("vision.lag.disappear", t_delta_disappear);
        SmartDashboard.putNumber("vision.lag.appear", t_delta_appear);
        done = true;
      }
    }
  }

  @Override
  public boolean isFinished() {
    return done;
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.visionSubsystem.turnRingLightOn();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}