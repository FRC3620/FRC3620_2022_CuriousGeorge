package frc.robot.subsystems;

import java.sql.Time;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics. CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;

import org.usfirst.frc3620.misc.RobotMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class turretSubsystem extends SubsystemBase {
  boolean encoderIsValid = false;
  CANSparkMax turretDrive = RobotContainer.turretSubsystemturretSpinner;
  RelativeEncoder turretEncoder = RobotContainer.turretSubsystemturretEncoder;
  SparkMaxPIDController turretPID = RobotContainer.turretSubsystemturretSpinner.getPIDController();
  Timer calibrationTimer;
  /**
   * Creates a new turretSubsystem.
   */
  public turretSubsystem() {
    turretEncoder.setPositionConversionFactor(90/7.8);
    turretEncoder.setVelocityConversionFactor(1);

    // set up PID for turretPID here
  turretPID.setP(0);   
  turretPID.setI(0.0);     
  turretPID.setD(30);    
  turretPID.setFF(0.0);      

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
        turnturret(-0.045);

        if (calibrationTimer == null) {
          calibrationTimer = new Timer();
          calibrationTimer.reset();
          calibrationTimer.start();
         } else {
          if (calibrationTimer.get() > 0.5){
            if (Math.abs(turretSpeed) < 20) {
              encoderIsValid = true;
              turnturret(0.0);
              turretEncoder.setPosition(0.0);
            }
          
          }
        }
      }
    }

    SmartDashboard.putNumber("turretSpeed", turretSpeed);
    SmartDashboard.putNumber("turretposition", turretPosition);
    SmartDashboard.putNumber("turretCurrent", turretCurrent);
    SmartDashboard.putBoolean("turretEncoderValid", encoderIsValid);
    SmartDashboard.putNumber("turretPower", turretPower);
    SmartDashboard.putNumber("turretVelocityConversionFactor", turretEncoder.getVelocityConversionFactor());
  }

  /**
   * 
   * @param speed speed to turn it. positive is clockwise
   */
  public void turnturret(double speed) {
    turretDrive.set(speed);
  }

  public void setturretPosition (double angle) {
    turretPID.setReference(angle, ControlType.kPosition);
  }
}
