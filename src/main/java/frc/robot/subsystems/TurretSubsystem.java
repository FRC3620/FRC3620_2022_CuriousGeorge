package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;

import org.usfirst.frc3620.misc.RobotMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class TurretSubsystem extends SubsystemBase {
  boolean encoderIsValid = false;
  CANSparkMax turretDrive = RobotContainer.turretSubsystemturretSpinner;
  RelativeEncoder turretEncoder = RobotContainer.turretSubsystemturretEncoder;
  SparkMaxPIDController turretPID = RobotContainer.turretSubsystemturretSpinner.getPIDController();
  Timer calibrationTimer;
  /**
   * Creates a new turretSubsystem.
   */
  public TurretSubsystem() {
    turretEncoder.setPositionConversionFactor(90/7.8);
    turretEncoder.setVelocityConversionFactor(1);

    // set up PID for turretPID here
    turretPID.setP(0.09);   //0.09
    turretPID.setI(0);     //0.0
    turretPID.setD(30);    //30
    turretPID.setFF(0.0);      //0.0
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double turretSpeed = turretEncoder.getVelocity();
    double turretPosition = turretEncoder.getPosition();
    double turretCurrent = turretDrive.getOutputCurrent();
    double turretPower = turretDrive.getAppliedOutput();

    if(Robot.getCurrentRobotMode() == RobotMode.TELEOP || Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS){
      if (!encoderIsValid) {
        turnTurret(-0.045);

        if (calibrationTimer == null) {
          calibrationTimer = new Timer();
          calibrationTimer.reset();
          calibrationTimer.start();
        } else {
          if (calibrationTimer.get() > 0.5){
            if (Math.abs(turretSpeed) < 20) {
              encoderIsValid = true;
              turnTurret(0.0);
              turretEncoder.setPosition(0.0);
            }
          }
        }
      }
    }

    SmartDashboard.putNumber("turretSpeed", turretSpeed);
    SmartDashboard.putNumber("turretPosition", turretPosition);
    SmartDashboard.putNumber("turretCurrent", turretCurrent);
    SmartDashboard.putBoolean("turretEncoderValid", encoderIsValid);
    SmartDashboard.putNumber("turretPower", turretPower);
    SmartDashboard.putNumber("turretVelocityConversionFactor", turretEncoder.getVelocityConversionFactor());
  }

  /**
   * 
   * @param speed speed to turn it. positive is clockwise
   */
  public void turnTurret(double speed) {
    turretDrive.set(speed);
  }

  public void setTurretPosition (double angle) {
    if (encoderIsValid) {
    if(angle<0) angle = 0;
    if(angle>220) angle =220;
      turretPID.setReference(angle, ControlType.kPosition);
      SmartDashboard.putNumber("turretRequestedAngle", angle);
    }
  }
  public double getCurrentTurretPosition(){
    return turretEncoder.getPosition();

  }
}
