package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

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
  SparkMaxPIDController turretPID = null;
  Timer calibrationTimer;

  double requestedTurretPosition;
  /**
   * Creates a new turretSubsystem.
   */
  public TurretSubsystem() {
    if (turretDrive != null) {
      SendableRegistry.addLW(turretDrive, getName(), "turret drive");

      turretPID = turretDrive.getPIDController();

      // set up PID for turretPID here
      turretPID.setP(0.09);   //0.09
      turretPID.setI(0.0);     //0.0
      turretPID.setD(30);    //30
      turretPID.setFF(0.0);      //0.0

      turretPID.setOutputRange(-0.60, 0.60);
    }

    if (turretEncoder != null) {
      turretEncoder.setPositionConversionFactor(90.0/115.0);
      turretEncoder.setVelocityConversionFactor(1);
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
      SmartDashboard.putNumber("turretCurrent", turretCurrent);
      SmartDashboard.putNumber("turretPower", turretPower);

      if (turretEncoder != null) {
        double turretSpeed = turretEncoder.getVelocity();
        double turretPosition = turretEncoder.getPosition();
        SmartDashboard.putNumber("turretSpeed", turretSpeed);
        SmartDashboard.putNumber("turretPosition", turretPosition);
        SmartDashboard.putNumber("turretVelocityConversionFactor", turretEncoder.getVelocityConversionFactor());
  
        if(Robot.getCurrentRobotMode() == RobotMode.TELEOP || Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS){
          if (!encoderIsValid) {
            turnTurret(0.06); //turns turret clockwise

            if (calibrationTimer == null) {
              calibrationTimer = new Timer();
              calibrationTimer.reset();
              calibrationTimer.start();
            } else {
              if (calibrationTimer.get() > 0.5){
                if (Math.abs(turretSpeed) < 20) {
                  encoderIsValid = true;
                  turnTurret(0.0);
                  turretEncoder.setPosition(270.0); //check encoder position
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

  /**
   * 
   * @param speed speed to turn it. positive is clockwise
   */
  public void turnTurret(double speed) {
    turretDrive.set(speed);
  }

  public void setTurretPosition (double angle) {
    logger.info("setTurretPosition set to {} by {}", angle, EventLogging.callerName());
    if(angle < -45) {
      angle = -45;
    }
    if(angle > 265) {
      angle = 265;
    }
    SmartDashboard.putNumber("turretRequestedAngle", angle);
    requestedTurretPosition = angle;
    if (encoderIsValid) {
      turretPID.setReference(angle, ControlType.kPosition);
    }
  }

  public double getRequestedTurretPosition(){
    return requestedTurretPosition;
  }

  public double getCurrentTurretPosition(){
    return turretEncoder.getPosition();
  }
}
