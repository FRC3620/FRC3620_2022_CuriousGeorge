// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeOffCommand;
import frc.robot.miscellaneous.CANSparkMaxSendable;

public class IntakeSubsystem extends SubsystemBase {
  CANSparkMaxSendable intakeWheelbar = RobotContainer.intakeWheelbar;
  CANSparkMaxSendable intakeBelt = RobotContainer.intakeBelt;

  Command currentCommand = null;
  Command previousCommand = null;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    if (intakeWheelbar != null) {
      SendableRegistry.addLW(intakeWheelbar, getName(), "intake wheelbar");
    }
    if (intakeBelt != null) {
      SendableRegistry.addLW(intakeBelt, getName(), "intake belt");
    }

    setDefaultCommand(new IntakeOffCommand(this));
  }

  @Override
  public void periodic() {
    Command runningNow = getCurrentCommand();
    if (runningNow != currentCommand) {
      previousCommand = currentCommand;
      currentCommand = runningNow;
    }
  }

  public void startPreviousCommand() {
    if (previousCommand != null) {
      previousCommand.schedule();
    }
  }

   /**
   * Spin the intake wheel and intake belt.
   * @param speed how fast to spin. positive is inward, negative is outward.
   */
  public void spinIntakeWheelBar(double speed) {
    if (intakeWheelbar != null) {
      intakeWheelbar.set(speed);
     }  
    }

  public void spinIntakeBelt(double speed) {
    if (intakeBelt != null) {
      intakeBelt.set(speed);
    }
  }

  public double getIntakeBeltSpeed() {
    if (intakeBelt != null) {
      return intakeBelt.get();
    }
    return 0;
  }

  public double getIntakeWheelbarSpeed() {
    if (intakeWheelbar != null) {
      return intakeWheelbar.get();
    }
    return 0;
  }
}