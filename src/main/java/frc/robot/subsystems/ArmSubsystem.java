package frc.robot.subsystems;

import org.slf4j.Logger;

import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ArmSubsystem extends SubsystemBase {
  
	Logger logger = EventLogging.getLogger(getClass(), Level.INFO);
  DoubleSolenoid climberArmTilt = RobotContainer.climberArmTilt;
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    if (climberArmTilt != null) {
      logger.info("have climberArmTilt");
      SendableRegistry.addLW(climberArmTilt, getName(), "tilt");
    } else {
      logger.info("missing climberArmTilt");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climberArmTiltIn() {
    if (climberArmTilt != null) { 
      climberArmTilt.set(Value.kReverse);
    }
  }

  public void climberArmTiltOut() {
    if (climberArmTilt != null) { 
      climberArmTilt.set(Value.kForward);
    }
  }

  public void climberArmTiltOff() {
    if (climberArmTilt != null) { 
      climberArmTilt.set(Value.kOff);
    }
  }
}