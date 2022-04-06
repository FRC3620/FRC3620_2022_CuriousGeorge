package frc.robot;

import org.usfirst.frc3620.logger.DataLogger;
import org.usfirst.frc3620.misc.CANDeviceFinder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.miscellaneous.ShooterCalculator;
import frc.robot.subsystems.VisionSubsystem;

import java.text.DecimalFormat;

public class RobotDataLogger {
	PowerDistribution powerDistribution = null;
	private final boolean logDriveMotorCurrent = true;

	public RobotDataLogger (DataLogger dataLogger, CANDeviceFinder canDeviceFinder) {
		dataLogger.addDataProvider("matchTime", () -> f2(DriverStation.getMatchTime()));
		dataLogger.addDataProvider("robotMode", () -> Robot.currentRobotMode.toString());
		dataLogger.addDataProvider("robotModeInt", () -> Robot.currentRobotMode.ordinal());
		dataLogger.addDataProvider("batteryVoltage", () -> f2(RobotController.getBatteryVoltage()));
		dataLogger.addDataProvider("can.utilization", () -> f2(RobotController.getCANStatus().percentBusUtilization));
		dataLogger.addDataProvider("can.receive_errors", () -> RobotController.getCANStatus().receiveErrorCount);
		dataLogger.addDataProvider("can.transmit_errors", () -> RobotController.getCANStatus().transmitErrorCount);

		if (canDeviceFinder.isPowerDistributionPresent()) {
			powerDistribution = new PowerDistribution();
			dataLogger.addDataProvider("pdp.totalCurrent", () -> f2(powerDistribution.getTotalCurrent()));
			dataLogger.addDataProvider("pdp.totalPower", () -> f2(powerDistribution.getTotalPower()));
			dataLogger.addDataProvider("pdp.totalEnergy", () -> f2(powerDistribution.getTotalEnergy()));
		}

		dataLogger.addDataProvider("vision.target.data_is_fresh", () -> RobotContainer.visionSubsystem.isTargetDataStale() ? 0 : 1);
		dataLogger.addDataProvider("vision.target.found", () -> RobotContainer.visionSubsystem.isTargetFound() ? 1 : 0);
		dataLogger.addDataProvider("vision.target.centered", () -> RobotContainer.visionSubsystem.isTargetCentered() ? 1 : 0);
		//dataLogger.addDataProvider("vision.target.x", () -> RobotContainer.visionSubsystem.getTargetXLocation());
		dataLogger.addDataProvider("vision.target.y", () -> RobotContainer.visionSubsystem.getTargetYLocation());
		dataLogger.addDataProvider("vision.range", () -> f2(calculateRange()));
		dataLogger.addDataProvider("vision.cargo.x", () -> RobotContainer.visionSubsystem.getBallXLocation());
		dataLogger.addDataProvider("vision.cargo.y", () -> RobotContainer.visionSubsystem.getBallYLocation());

		dataLogger.addDataProvider("navx.absolute", () -> f2(RobotContainer.driveSubsystem.getNavXAbsoluteAngle()));
		dataLogger.addDataProvider("navx.fixed", () -> f2(RobotContainer.driveSubsystem.getNavXFixedAngle()));
		dataLogger.addDataProvider("navx.offset", () -> f2(RobotContainer.driveSubsystem.getNavXOffset()));

		if (RobotContainer.climberExtentionMotor != null) {
			dataLogger.addDataProvider("climber.power", () -> f2(RobotContainer.climberExtentionMotor.getAppliedOutput()));
			dataLogger.addDataProvider("climber.current", () -> f2(RobotContainer.climberExtentionMotor.getOutputCurrent()));
			dataLogger.addDataProvider("climber.temperature", () -> f2(RobotContainer.climberExtentionMotor.getMotorTemperature()));
		}

		if (RobotContainer.driveSubsystemLeftFrontDrive != null) {
			dataLogger.addDataProvider("drive.lf.az.home_encoder", () -> f2(RobotContainer.driveSubsystemLeftFrontHomeEncoder.getVoltage()));
			dataLogger.addDataProvider("drive.lf.az.encoder", () -> f2(RobotContainer.driveSubsystemLeftFrontAzimuthEncoder.getPosition()));
			dataLogger.addDataProvider("drive.rf.az.home_encoder", () -> f2(RobotContainer.driveSubsystemRightFrontHomeEncoder.getVoltage()));
			dataLogger.addDataProvider("drive.rf.az.encoder", () -> f2(RobotContainer.driveSubsystemRightFrontAzimuthEncoder.getPosition()));
			dataLogger.addDataProvider("drive.lb.az.home_encoder", () -> f2(RobotContainer.driveSubsystemLeftBackHomeEncoder.getVoltage()));
			dataLogger.addDataProvider("drive.lb.az.encoder", () -> f2(RobotContainer.driveSubsystemLeftBackAzimuthEncoder.getPosition()));
			dataLogger.addDataProvider("drive.rb.az.home_encoder", () -> f2(RobotContainer.driveSubsystemRightBackHomeEncoder.getVoltage()));
			dataLogger.addDataProvider("drive.rb.az.encoder", () -> f2(RobotContainer.driveSubsystemRightBackAzimuthEncoder.getPosition()));

			if (logDriveMotorCurrent) {
				dataLogger.addDataProvider("drive.lf.drive_power", () -> f2(RobotContainer.driveSubsystemLeftFrontDrive.getAppliedOutput()));
				dataLogger.addDataProvider("drive.rf.drive_power", () -> f2(RobotContainer.driveSubsystemRightFrontDrive.getAppliedOutput()));
				dataLogger.addDataProvider("drive.lb.drive_power", () -> f2(RobotContainer.driveSubsystemLeftBackDrive.getAppliedOutput()));
				dataLogger.addDataProvider("drive.rb.drive_power", () -> f2(RobotContainer.driveSubsystemRightBackDrive.getAppliedOutput()));
				dataLogger.addDataProvider("drive.lf.drive_current", () -> f2(RobotContainer.driveSubsystemLeftFrontDrive.getOutputCurrent()));
				dataLogger.addDataProvider("drive.rf.drive_current", () -> f2(RobotContainer.driveSubsystemRightFrontDrive.getOutputCurrent()));
				dataLogger.addDataProvider("drive.lb.drive_current", () -> f2(RobotContainer.driveSubsystemLeftBackDrive.getOutputCurrent()));
				dataLogger.addDataProvider("drive.rb.drive_current", () -> f2(RobotContainer.driveSubsystemRightBackDrive.getOutputCurrent()));
			}
		}

		if (RobotContainer.shooterSubsystemMainShooter1 != null) {
			dataLogger.addDataProvider("shooter.main.requestedRPM", () -> f2(RobotContainer.shooterSubsystem.getTopStatus().getRequestedRPM()));
			dataLogger.addDataProvider("shooter.main.actualRPM", () -> f2(RobotContainer.shooterSubsystem.getTopStatus().getActualRPM()));
			dataLogger.addDataProvider("shooter.main.power", () -> f2(RobotContainer.shooterSubsystem.getTopStatus().getAppliedPower()));
			dataLogger.addDataProvider("shooter.main.current", () -> f2(RobotContainer.shooterSubsystem.getTopStatus().getStatorCurrent()));
		}

		if (RobotContainer.shooterSubsystemBackSpinShooter != null) {
			dataLogger.addDataProvider("shooter.backspin.requestedRPM", () -> f2(RobotContainer.shooterSubsystem.getBackStatus().getRequestedRPM()));
			dataLogger.addDataProvider("shooter.backspin.actualRPM", () -> f2(RobotContainer.shooterSubsystem.getBackStatus().getActualRPM()));
			dataLogger.addDataProvider("shooter.backspin.power", () -> f2(RobotContainer.shooterSubsystem.getBackStatus().getAppliedPower()));
			dataLogger.addDataProvider("shooter.backspin.current", () -> f2(RobotContainer.shooterSubsystem.getBackStatus().getStatorCurrent()));
		}

		if (RobotContainer.preShooterSubsystemPreShooter != null) {
			dataLogger.addDataProvider("shooter.preshooter.requestedRPM", () -> f2(RobotContainer.preShooterSubsystem.getPreshooterStatus().getRequestedRPM()));
			dataLogger.addDataProvider("shooter.preshooter.actualRPM", () -> f2(RobotContainer.preShooterSubsystem.getPreshooterStatus().getActualRPM()));
			dataLogger.addDataProvider("shooter.preshooter.power", () -> f2(RobotContainer.preShooterSubsystem.getPreshooterStatus().getAppliedPower()));
			dataLogger.addDataProvider("shooter.preshooter.current", () -> f2(RobotContainer.preShooterSubsystem.getPreshooterStatus().getStatorCurrent()));
		}

		dataLogger.addDataProvider("hood.requested_position", () -> f2(RobotContainer.shooterSubsystem.getRequestedHoodPosition()));
		dataLogger.addDataProvider("hood.actual_position", () -> f2(RobotContainer.shooterSubsystem.getHoodPosition()));
		if (RobotContainer.shooterSubsystemHoodMax != null) {
			dataLogger.addDataProvider("hood.power", () -> f2(RobotContainer.shooterSubsystemHoodMax.getAppliedOutput()));
			dataLogger.addDataProvider("hood.current", () -> f2(RobotContainer.shooterSubsystemHoodMax.getOutputCurrent()));
		}

		if (RobotContainer.turretSubsystemturretSpinner != null){
			dataLogger.addDataProvider("turret.power", () -> f2(RobotContainer.turretSubsystemturretSpinner.getAppliedOutput()));
			dataLogger.addDataProvider("turret.current", () -> f2(RobotContainer.turretSubsystemturretSpinner.getOutputCurrent()));
			dataLogger.addDataProvider("turret.requested_position", () -> f2(RobotContainer.turretSubsystem.getRequestedTurretPosition()));
			dataLogger.addDataProvider("turret.current_position", () -> f2(RobotContainer.turretSubsystem.getCurrentTurretPosition()));
		}

		if (RobotContainer.intakeBelt != null && RobotContainer.intakeWheelbar != null){
			dataLogger.addDataProvider("intake.belt.power", () -> f2(RobotContainer.intakeBelt.getAppliedOutput()));
			dataLogger.addDataProvider("intake.belt.current", () -> f2(RobotContainer.intakeBelt.getOutputCurrent()));
			dataLogger.addDataProvider("intake.wheelbar.power", () -> f2(RobotContainer.intakeWheelbar.getAppliedOutput()));
			dataLogger.addDataProvider("intake.wheelbar.current", () -> f2(RobotContainer.intakeWheelbar.getOutputCurrent()));
		}
	}

	double calculateRange() {
		double ty = RobotContainer.visionSubsystem.getTargetYLocation();
		double d = ShooterCalculator.calcDistanceFromHub(ty);
		return d;
	}

	private DecimalFormat f2formatter = new DecimalFormat("#.##");

	private String f2(double value) { return f2formatter.format(value);	}
}