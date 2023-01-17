package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.LoggingMaster;
import org.usfirst.frc3620.logger.EventLogging.Level;
import org.usfirst.frc3620.misc.RobotMode;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.miscellaneous.CANSparkMaxSendable;

public class TurretSubsystem extends SubsystemBase {
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

  boolean encoderIsValid = false;
  CANSparkMaxSendable turretDrive = RobotContainer.turretSubsystemturretSpinner;
  RelativeEncoder turretEncoder = RobotContainer.turretSubsystemturretEncoder;
  SparkMaxLimitSwitch turretForwardLimitSwitch;
  SparkMaxPIDController turretPID = null;
  Timer calibrationTimer;

  Double requestedTurretPositionWhileCalibrating = null;

  double requestedTurretPosition;
  /**
   * Creates a new turretSubsystem.
   */
  public TurretSubsystem() {
    if (turretDrive != null) {
      SendableRegistry.addLW(turretDrive, getName(), "turret drive");

      turretPID = turretDrive.getPIDController();

      // set up PID for turretPID here
      turretPID.setP(0.11);   //0.045
      turretPID.setI(0.0);     //0.0
      turretPID.setD(30);      //30
      turretPID.setFF(0.0);    //0.0

      turretPID.setOutputRange(-0.8, 0.8);

      turretForwardLimitSwitch = turretDrive.getForwardLimitSwitch(Type.kNormallyOpen);
      turretForwardLimitSwitch.enableLimitSwitch(true);
    }

    if (turretEncoder != null) {
      turretEncoder.setPositionConversionFactor(90.0/26.861);  //90.0/115.0
      turretEncoder.setVelocityConversionFactor(1);
    }
  }

  boolean isForwardLimitSwitchPressed() {
    if (turretForwardLimitSwitch != null) {
      return turretForwardLimitSwitch.isPressed();
    } else {
      return true;
    }
  }

  Command lastCommand = null;

  @Override
  public void periodic() {
    Command currentCommand = getCurrentCommand();
    if (currentCommand != lastCommand) {
      logger.info("new command: {} -> {}", lastCommand, currentCommand);
      lastCommand = currentCommand;
    }
    // This method will be called once per scheduler run
    if (turretDrive != null) {
      double turretCurrent = turretDrive.getOutputCurrent();
      double turretPower = turretDrive.getAppliedOutput();
      boolean turretForwardLimitSwitch = isForwardLimitSwitchPressed();

      SmartDashboard.putNumber("turretCurrent", turretCurrent);
      SmartDashboard.putNumber("turretPower", turretPower);
      SmartDashboard.putBoolean("turret forward limit", turretForwardLimitSwitch);

      if (turretEncoder != null) {
        double turretSpeed = turretEncoder.getVelocity();
        double turretPosition = turretEncoder.getPosition();
        SmartDashboard.putNumber("turretSpeed", turretSpeed);
        SmartDashboard.putNumber("turretPosition", turretPosition);
        SmartDashboard.putNumber("turretVelocityConversionFactor", turretEncoder.getVelocityConversionFactor());
  
        if(Robot.getCurrentRobotMode() == RobotMode.TELEOP || Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS){
          if (!encoderIsValid) {
            turnTurret(0.05); //turns turret clockwise

            if (calibrationTimer == null) {
              calibrationTimer = new Timer();
              calibrationTimer.reset();
              calibrationTimer.start();
            } else {
              boolean atTheLimit = false;
              if(isForwardLimitSwitchPressed()) {
                atTheLimit = true;
              }
              if (calibrationTimer.get() > 0.5 && Math.abs(turretSpeed) < 150 ){
                atTheLimit = true;
              }
              if (atTheLimit) {
                encoderIsValid = true;
                turnTurret(0.0);
                turretEncoder.setPosition(270.0);
                if (requestedTurretPositionWhileCalibrating != null) {
                  setTurretPosition(requestedTurretPositionWhileCalibrating);
                  requestedTurretPositionWhileCalibrating = null;
                }
              }
            }
          }
        } else {
          calibrationTimer = null; // start over!
        } 
      }
    }

    SmartDashboard.putBoolean("turretEncoderValid", encoderIsValid);
  }

  public boolean turretEncoderIsValid() {
    return encoderIsValid;
  }

  /**
   * 
   * @param speed speed to turn it. positive is clockwise
   */
  public void turnTurret(double speed) {
    turretDrive.set(speed);
  }

  public void setTurretPosition (double angle) {
    // logger.info("setTurretPosition set to {} by {}", angle, EventLogging.callerName());
    if(angle < -45) {
      angle = -45;
    }
    if(angle > 260) {
      angle = 260;
    }
    SmartDashboard.putNumber("turretRequestedAngle", angle);
    requestedTurretPosition = angle;
    if (encoderIsValid) {
      if (! Double.isNaN(requestedTurretPosition)) {
        turretPID.setReference(angle, ControlType.kPosition);
      }
    } else {
      requestedTurretPositionWhileCalibrating = angle;
    }
  }

  public double getRequestedTurretPosition(){
    return requestedTurretPosition;
  }

  public double getCurrentTurretPosition(){
    if(turretEncoder != null){
      return turretEncoder.getPosition();
    } else {
      return requestedTurretPosition;
    }
  }
}