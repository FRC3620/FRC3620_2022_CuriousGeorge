// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class LightSubsystem extends SubsystemBase {
  /** Creates a new LightSubsystem. */
  public LightSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double remainingMatchTime = DriverStation.getMatchTime();
    SmartDashboard.putNumber("remainingMatchTime", remainingMatchTime);
    if(DriverStation.isTeleopEnabled()){
      if (remainingMatchTime < 45){
        RobotContainer.redLight.set(true);
        RobotContainer.greenLight.set(false);
      }
      else if (remainingMatchTime < 60){
        RobotContainer.greenLight.set(true);
        RobotContainer.redLight.set(true);
      } 
      else{
        RobotContainer.redLight.set(false);
        RobotContainer.greenLight.set(true);
      }
    }
    else{
      RobotContainer.greenLight.set(false);
      RobotContainer.redLight.set(false);
    }
    
  }
}
