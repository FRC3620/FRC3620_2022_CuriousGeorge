// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

 

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable networkTable = inst.getTable("V/Cargo");
  
  private NetworkTableEntry ballX = networkTable.getEntry("ball.x");
  private  NetworkTableEntry allianceColor = networkTable.getEntry("color");

  private NetworkTable networkTable2= inst.getTable("V/Target");
  private NetworkTableEntry targetX = networkTable2.getEntry("target.x");
  /** Creates a new VisionSubSystem. */
  public VisionSubsystem() {

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(DriverStation.getAlliance() == DriverStation.Alliance.Red){
      allianceColor.setString("red");
    } else {
      allianceColor.setString("blue");
    }

    SmartDashboard.putNumber("vision.target.x", targetX.getDouble(-2));
  }

  public double getBallXLocation(){
    return ballX.getDouble(-1);
  }

  public double getTargetXLocation(){
    double rv = targetX.getDouble(-3);

    return rv;
  }
}
