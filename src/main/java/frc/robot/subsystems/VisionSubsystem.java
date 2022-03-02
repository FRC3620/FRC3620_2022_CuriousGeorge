package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class VisionSubsystem extends SubsystemBase {
  public static final String VisionLightOnCommand = null;
  public final String turnVisionLightOn = null;
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable networkTable = inst.getTable("V/Cargo");
  
  private NetworkTableEntry ballX = networkTable.getEntry("ball.x");
  private NetworkTableEntry ballY = networkTable.getEntry("ball.y");
  private NetworkTableEntry allianceColor = networkTable.getEntry("color");

  private NetworkTable networkTable2= inst.getTable("V/Target");
  private NetworkTableEntry targetX = networkTable2.getEntry("target.x");
  private Solenoid visionLight = RobotContainer.ringLight;
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    SendableRegistry.addLW(RobotContainer.ringLight, getName(), "ringlight");
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
  
  public double getBallYLocation(){
    return ballY.getDouble(-1);
  }
  public void turnVisionLightOn() {
    visionLight.set(true);
  }
}