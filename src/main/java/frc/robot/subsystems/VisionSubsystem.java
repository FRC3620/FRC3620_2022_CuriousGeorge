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

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry tl = table.getEntry("tl");

  double targetDataLastUpdated = 0;

  private Solenoid ringLight = RobotContainer.ringLight;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    SendableRegistry.addLW(RobotContainer.ringLight, getName(), "ringlight");

    tl.addListener(new LimelightListener(), EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);
  }

  class LimelightListener implements Consumer<EntryNotification> {
    @Override
    public void accept(EntryNotification t) {
      targetDataLastUpdated = Timer.getFPGATimestamp();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      allianceColor.setString("red");
    } else {
      allianceColor.setString("blue");
    }

    double targetYLocation = getTargetYLocation();
    double targetDistance = ShooterCalculator.calcDistanceFromHub(targetYLocation);
    double targetRPM = ShooterCalculator.calcMainRPM(targetDistance);
    double targetHood = ShooterCalculator.calcHoodAngle(targetDistance);
    SmartDashboard.putNumber("vision.target.data_age", getTargetDataAge());
    SmartDashboard.putBoolean("vision.target.data_is_fresh", !isTargetDataStale());
    SmartDashboard.putNumber("vision.target.y", targetYLocation);
    SmartDashboard.putNumber("vision.calculated.distance", targetDistance);
    SmartDashboard.putNumber("vision.calculated.RPM", targetRPM);
    SmartDashboard.putNumber("vision.calculated.hood", targetHood);
    SmartDashboard.putBoolean("vision.target.found", isTargetFound());
    SmartDashboard.putBoolean("vision.target.centered", isTargetCentered());

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
    if (isTargetDataStale()) {
      rv = false;
    }
    return rv;
  }

  public double getTargetYLocation(){
    if (!isTargetFound()) return Double.NaN;
    return ty.getDouble(0.0);
  }

  public double getTargetXDegrees() {
    if (!isTargetFound()) return Double.NaN;
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
    if (Double.isNaN(ylocation)) {
      return false;
    }
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

  public void turnRingLightOn() {
    if (ringLight != null) {
      ringLight.set(true);
    }
  }

  public void turnRingLightOff() {
    if (ringLight != null) {
      ringLight.set(false);
    }
  }
}