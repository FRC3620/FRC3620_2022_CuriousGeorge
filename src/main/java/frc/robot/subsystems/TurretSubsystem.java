package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import org.usfirst.frc3620.misc.RobotMode;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.miscellaneous.CANSparkMaxSendable;

public class TurretSubsystem extends SubsystemBase {
  boolean encoderIsValid = false;
  CANSparkMaxSendable turretDrive = RobotContainer.turretSubsystemturretSpinner;
  RelativeEncoder turretEncoder = RobotContainer.turretSubsystemturretEncoder;
  SparkMaxPIDController turretPID = null;
  Timer calibrationTimer;
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
    }

    if (turretEncoder != null) {
      turretEncoder.setPositionConversionFactor(90/7.8);
      turretEncoder.setVelocityConversionFactor(1);
    }

  }

  @Override
  public void periodic() {
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
            turnTurret(0.045); //turns turret clockwise

            if (calibrationTimer == null) {
              calibrationTimer = new Timer();
              calibrationTimer.reset();
              calibrationTimer.start();
            } else {
              if (calibrationTimer.get() > 0.5){
                if (Math.abs(turretSpeed) < 20) {
                  encoderIsValid = true;
                  turnTurret(0.0);
                  turretEncoder.setPosition(0.0); //check encoder position
                }
              }
            }
          }
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
    SmartDashboard.putNumber("turretRequestedAngle", angle);
    if (encoderIsValid) {
      if(angle < -45) angle = -45;
      if(angle > 270) angle = 270;
      turretPID.setReference(angle, ControlType.kPosition);
    }
  }
  public double getCurrentTurretPosition(){
    return turretEncoder.getPosition();
  }
}
