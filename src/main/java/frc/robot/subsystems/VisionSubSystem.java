// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubSystem extends SubsystemBase {

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable networkTable = inst.getTable("V/Cargo");
  
  private NetworkTableEntry ballX = networkTable.getEntry("ball.x");
  private NetworkTableEntry ballY = networkTable.getEntry("ball.y");
  private  NetworkTableEntry allianceColor = networkTable.getEntry("color");
  /** Creates a new VisionSubSystem. */
  public VisionSubSystem() {

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(DriverStation.getAlliance() == DriverStation.Alliance.Red){
      allianceColor.setString("red");
    } else {
      allianceColor.setString("blue");
    }
  }

  public double getBallXLocation(){
    return ballX.getDouble(-1);
  }

  public double getBallYLocation(){
    return ballY.getDouble(-1);
  }
}
