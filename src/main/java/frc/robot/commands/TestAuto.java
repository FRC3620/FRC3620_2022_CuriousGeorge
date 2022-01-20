
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
      //shoot
      new AutoDriveCommand(4*12, 0, .5, 180, driveSubsystem)
      //,
      //new AutoDriveCommand(80, 60 , .5, 240, driveSubsystem),
      //shoot
      //new AutoDriveCommand(120, 100, .3, 260, driveSubsystem)
      //shoot
      //new AutoDriveCommand(4*12, 270, .2, 180, driveSubsystem),
      );
  }
}