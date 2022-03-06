package frc.robot;

import org.usfirst.frc3620.logger.DataLogger;
import org.usfirst.frc3620.misc.CANDeviceFinder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;

public class RobotDataLogger {
	PowerDistribution powerDistribution = null;

	public RobotDataLogger (DataLogger dataLogger, CANDeviceFinder canDeviceFinder) {

		dataLogger.addDataProvider("matchTime", () -> DataLogger.f2(DriverStation.getMatchTime()));
		dataLogger.addDataProvider("robotMode", () -> Robot.currentRobotMode.toString());
		dataLogger.addDataProvider("robotModeInt", () -> Robot.currentRobotMode.ordinal());
		dataLogger.addDataProvider("batteryVoltage", () -> DataLogger.f2(RobotController.getBatteryVoltage()));
		dataLogger.addDataProvider("can.utilization", () -> DataLogger.f2(RobotController.getCANStatus().percentBusUtilization));
		dataLogger.addDataProvider("can.receive_errors", () -> DataLogger.f2(RobotController.getCANStatus().receiveErrorCount));
		dataLogger.addDataProvider("can.transmit_errors", () -> DataLogger.f2(RobotController.getCANStatus().transmitErrorCount));

		if (canDeviceFinder.isPowerDistributionPresent()) {
			powerDistribution = new PowerDistribution();
			dataLogger.addDataProvider("pdp.totalCurrent", () -> DataLogger.f2(powerDistribution.getTotalCurrent()));
			dataLogger.addDataProvider("pdp.totalPower", () -> DataLogger.f2(powerDistribution.getTotalPower()));
			dataLogger.addDataProvider("pdp.totalEnergy", () -> DataLogger.f2(powerDistribution.getTotalEnergy()));
		}

		if (RobotContainer.climberExtentionMotor != null) {
			dataLogger.addDataProvider("climber.power", () -> DataLogger.f2(RobotContainer.climberExtentionMotor.getAppliedOutput()));
			dataLogger.addDataProvider("climber.current", () -> DataLogger.f2(RobotContainer.climberExtentionMotor.getOutputCurrent()));
			dataLogger.addDataProvider("climber.temperature", () -> DataLogger.f2(RobotContainer.climberExtentionMotor.getMotorTemperature()));
		}

		if (RobotContainer.driveSubsystemLeftFrontDrive != null) {
			dataLogger.addDataProvider("drive.lf.az.home_encoder", () -> DataLogger.f2(RobotContainer.driveSubsystemLeftFrontHomeEncoder.getVoltage()));
			dataLogger.addDataProvider("drive.lf.az.encoder", () -> DataLogger.f2(RobotContainer.driveSubsystemLeftFrontAzimuthEncoder.getPosition()));
			dataLogger.addDataProvider("drive.rf.az.home_encoder", () -> DataLogger.f2(RobotContainer.driveSubsystemRightFrontHomeEncoder.getVoltage()));
			dataLogger.addDataProvider("drive.rf.az.encoder", () -> DataLogger.f2(RobotContainer.driveSubsystemRightFrontAzimuthEncoder.getPosition()));
			dataLogger.addDataProvider("drive.lb.az.home_encoder", () -> DataLogger.f2(RobotContainer.driveSubsystemLeftBackHomeEncoder.getVoltage()));
			dataLogger.addDataProvider("drive.lb.az.encoder", () -> DataLogger.f2(RobotContainer.driveSubsystemLeftBackAzimuthEncoder.getPosition()));
			dataLogger.addDataProvider("drive.rb.az.home_encoder", () -> DataLogger.f2(RobotContainer.driveSubsystemRightBackHomeEncoder.getVoltage()));
			dataLogger.addDataProvider("drive.rb.az.encoder", () -> DataLogger.f2(RobotContainer.driveSubsystemRightBackAzimuthEncoder.getPosition()));
		}
	}
}
