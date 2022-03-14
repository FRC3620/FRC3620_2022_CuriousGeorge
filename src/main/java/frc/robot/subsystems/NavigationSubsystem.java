// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class NavigationSubsystem extends SubsystemBase {
  /** Creates a new NavigationSubsystem. */
  public NavigationSubsystem() {}

  @Override
  public void periodic() {
    PositionEstimate positionEstimate = RobotContainer.driveSubsystem.getNavXDisplacement();
    SmartDashboard.putNumber("nav.navx.x", positionEstimate.x);
    SmartDashboard.putNumber("nav.navx.y", positionEstimate.y);
    SmartDashboard.putNumber("nax.navx.z", positionEstimate.z);
    SmartDashboard.putNumber("nav.navx.time_since_reset", Timer.getFPGATimestamp() - positionEstimate.timeOfReset);
    SmartDashboard.putNumber("nav.navx.position_age", Timer.getFPGATimestamp() - positionEstimate.when);
    // This method will be called once per scheduler run
  }

  public static class PositionEstimate {
    double x, y, z, when, timeOfReset;
    public PositionEstimate (double x, double y, double z, double timeOfReset) {
      this.x = x;
      this.y = y;
      this.z = z;
      this.timeOfReset = timeOfReset;
      this.when = Timer.getFPGATimestamp();
    }
  }
}
