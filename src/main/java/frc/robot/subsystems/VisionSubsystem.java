package frc.robot.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Consumer;

import com.google.gson.Gson;
import com.google.gson.annotations.SerializedName;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import frc.robot.RobotContainer;
import frc.robot.miscellaneous.ShooterCalculator;

public class VisionSubsystem extends SubsystemBase {
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable cargoNetworkTable = inst.getTable("V/Cargo");
  
  private NetworkTableEntry ballX = cargoNetworkTable.getEntry("ball.x");
  private NetworkTableEntry ballY = cargoNetworkTable.getEntry("ball.y");
  private NetworkTableEntry allianceColor = cargoNetworkTable.getEntry("color");

  private NetworkTable targetNetworkTable = inst.getTable("V/Target");
  private NetworkTableEntry nt_target_json = targetNetworkTable.getEntry("json");
  //private Solenoid visionLight = RobotContainer.ringLight;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry tv = table.getEntry("tv");

  Gson targetGson = new Gson();
  TargetData targetData = new TargetData();
  double targetDataLastUpdated = 0;

  class TargetData {
    Double tx, ty;

    @SerializedName("b")
    int boxes;

    @SerializedName("c")
    Integer frame_check_count;

    @SerializedName("f")
    boolean found;
  }

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    //SendableRegistry.addLW(RobotContainer.ringLight, getName(), "ringlight");

    String json = nt_target_json.getString(null);
    if (json != null) {
      updateTargetInfoFromTargetJson(json);
    }

    //turnVisionLightOn();

    nt_target_json.addListener(new TargetJsonListener(), EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);
  }

  class TargetJsonListener implements Consumer<EntryNotification> {
    @Override
    public void accept(EntryNotification t) {
      String json = t.value.getString();
      if (updateTargetInfoFromTargetJson(json)) {
        targetDataLastUpdated = Timer.getFPGATimestamp();
      }
    }
  }

  boolean updateTargetInfoFromTargetJson(String json) {
    boolean rv = false;
    try {
      targetData = targetGson.fromJson(json, TargetData.class);
      SmartDashboard.putNumber("vision.target.xdegrees", getTargetXDegrees());
      //SmartDashboard.putNumber("vision.target.x", getTargetXLocation());
      SmartDashboard.putNumber("vision.target.y", getTargetYLocation());
      SmartDashboard.putNumber("vision.target.boxes", targetData.boxes);
      SmartDashboard.putBoolean("vision.target.found", isTargetFound());
      SmartDashboard.putBoolean("vision.target.centered", isTargetCentered());
      rv = true;
    } catch (Exception ex) {
      logger.error ("trouble with parsing JSON?", ex);
    }
    return rv;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      allianceColor.setString("red");
    } else {
      allianceColor.setString("blue");
    }
    double targetDistance = ShooterCalculator.calcDistanceFromHub(getTargetYLocation());
    double targetRPM = ShooterCalculator.calcMainRPM(targetDistance);
    SmartDashboard.putNumber("vision.target.data_age", getTargetDataAge());
    SmartDashboard.putBoolean("vision.target.data_is_fresh", !isTargetDataStale());
    SmartDashboard.putNumber("vision.calculated.distance", targetDistance);
    SmartDashboard.putNumber("vision.calculated.RPM", targetRPM);

    //Drew playing with limelight 
    
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    
    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 27.0; //24.7
    
    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 36.0;
    
    // distance from the target to the floor
    double goalHeightInches = 105;
    
    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    
    
    //calculate distance
    double distanceFromGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians) + 24;
    SmartDashboard.putNumber("wow.It.Actually.Works.them", (distanceFromGoalInches / 12.0) + 0.7);

    double distanceFromGoalFeet = 15.63801 + -0.62701*targetOffsetAngle_Vertical + 0.013668*(targetOffsetAngle_Vertical*targetOffsetAngle_Vertical);
    //Camera distance from table is 2.25 inches.
    SmartDashboard.putNumber("wow.It.Actually.Works.us", distanceFromGoalFeet);
  }

  public boolean isTargetFound() {
    double currentTurretPosition = RobotContainer.turretSubsystem.getCurrentTurretPosition();

    boolean rv = true;
    if (currentTurretPosition > 31 && currentTurretPosition < 82) {
      // we are probably looking at our armpit
      rv = false;
    }
    if (tv.getDouble(0.0) == 0) {
      rv = false;
    }
    return rv;
  }

  /*public double getTargetXLocation(){
    if (!isTargetFound()) return Double.NaN;
    return targetData.x;
  }*/

  public double getTargetYLocation(){
    if (!isTargetFound()) return Double.NaN;
    //return targetData.y;
    return ty.getDouble(0.0);
  }

  public double getTargetXDegrees() {
    if (!isTargetFound()) return Double.NaN;
    // 5.0 / 0.08.25 was the original
    // 20.0 / 27.0 was an empirical correction
    //double k = (5.0 / 0.0825) * (20.0 / 27.0);
    //return (targetData.x - 0.5) * k;
    return tx.getDouble(0.0);
  }

  /**
   * is target data stale
   * @return 
   */
  public boolean isTargetDataStale() {
    return getTargetDataAge() > 0.5;
  }

  /**
   * how long since we got target data from rPi?
   * @return target data age in seconds
   */
  public double getTargetDataAge() {
    return Timer.getFPGATimestamp() - targetDataLastUpdated;
  }

  public boolean isTargetCentered() {
    if (!isTargetFound()) {
      return false;
    }
    double ylocation = getTargetYLocation();
    double distance = ShooterCalculator.calcDistanceFromHub(ylocation);
    double limit = 1;
    if (distance < 15) {
      limit = 5;
    }
    if (Math.abs(getTargetXDegrees()) < limit){
      return true;
    }
    return false;
  }

  public double getBallXLocation(){
    return ballX.getDouble(-1);
  }
  
  public double getBallYLocation(){
    return ballY.getDouble(-1);
  }

  /*public void turnVisionLightOn() {
    if (visionLight != null) {
      visionLight.set(true);
    }
  }

  public void turnVisionLightOff() {
    if (visionLight != null) {
      visionLight.set(false);
    }
  }*/
}