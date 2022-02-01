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
		if(RobotContainer.shooterSubsystemFalcon1 != null){
			dataLogger.addDataProvider("Shooter.Hood.Position.Requested", () -> (RobotContainer.shooterSubsystem.getRequestedHoodPosition()));
			dataLogger.addDataProvider("Shooter.Hood.Position.Actual", () -> (RobotContainer.shooterSubsystem.getActualHoodPosition()));
			dataLogger.addDataProvider("Shooter.Hood.CurrentOut", () -> (RobotContainer.shooterSubsystem.getHoodCurrent()));
			dataLogger.addDataProvider("Shooter.Hood.PercentOut", () -> (RobotContainer.shooterSubsystem.getHoodPercentOut()));
			dataLogger.addDataProvider("Shooter.Hood.Voltage", () -> (RobotContainer.shooterSubsystem.getHoodVoltage()));
	
			dataLogger.addDataProvider("Shooter.Top.Velocity.Requested", () -> (RobotContainer.shooterSubsystem.getRequestedTopShooterVelocity()));
			dataLogger.addDataProvider("Shooter.Top.Velocity.Actual", () -> (RobotContainer.shooterSubsystem.getActualTopShooterVelocity()));
			dataLogger.addDataProvider("Shooter.Top.CurrentOut", () -> (RobotContainer.shooterSubsystem.getTopShooterCurrent()));
			dataLogger.addDataProvider("Shooter.Top.PercentOut", () -> (RobotContainer.shooterSubsystem.getTopPercentOut()));
			dataLogger.addDataProvider("Shooter.Top.Voltage", () -> (RobotContainer.shooterSubsystem.getTopVoltage()));

			dataLogger.addDataProvider("Shooter.Bottom.Velocity.Requested", () -> (RobotContainer.shooterSubsystem.getRequestedBottomShooterVelocity()));
			dataLogger.addDataProvider("Shooter.Bottom.Velocity.Actual", () -> (RobotContainer.shooterSubsystem.getActualBottomShooterVelocity()));
			dataLogger.addDataProvider("Shooter.Bottom.CurrentOut", () -> (RobotContainer.shooterSubsystem.getBottomShooterCurrent()));
			dataLogger.addDataProvider("Shooter.Bottom.PercentOut", () -> (RobotContainer.shooterSubsystem.getBottomPercentOut()));
			dataLogger.addDataProvider("Shooter.Bottom.Voltage", () -> (RobotContainer.shooterSubsystem.getBottomVoltage()));

	
	
	}
}
}