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

		if (canDeviceFinder.isPowerDistributionPresent()) {
			powerDistribution = new PowerDistribution();
			dataLogger.addDataProvider("pdp.totalCurrent", () -> DataLogger.f2(powerDistribution.getTotalCurrent()));
			dataLogger.addDataProvider("pdp.totalPower", () -> DataLogger.f2(powerDistribution.getTotalPower()));
			dataLogger.addDataProvider("pdp.totalEnergy", () -> DataLogger.f2(powerDistribution.getTotalEnergy()));
		}

		if (RobotContainer.climberExtentionMotor != null) {
			dataLogger.addDataProvider("climber.power", () -> DataLogger.f2(RobotContainer.climberExtentionMotor.getAppliedOutput()));
			dataLogger.addDataProvider("climber.current", () -> DataLogger.f2(RobotContainer.climberExtentionMotor.getOutputCurrent()));
		}

		dataLogger.addDataProvider("vision.target_data_age", () -> DataLogger.f2(RobotContainer.visionSubsystem.getTargetDataAge()));
	}
}
