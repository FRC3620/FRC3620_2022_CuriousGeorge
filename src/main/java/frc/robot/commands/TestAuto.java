
    /*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TestAuto extends SequentialCommandGroup {
  /**
   * Creates a new SimpleAutoCommand.
   */
  public TestAuto(DriveSubsystem driveSubsystem) {
      addCommands(
      new AutoDriveCommand(4*12, 180, .2, 90, driveSubsystem),
      new AutoDriveCommand(4*12, 90, .2, 180, driveSubsystem),
      new AutoDriveCommand(4*12, 0, .3, 180, driveSubsystem),
      new AutoDriveCommand(4*12, 270, .2, 180, driveSubsystem)
      );
  }
}