package frc.robot;

import frc.robot.miscellaneous.MotorStatus;
import frc.robot.miscellaneous.ShooterCalculator;
import frc.robot.subsystems.PreShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import org.usfirst.frc3620.logger.FastDataLoggerCollections;
import org.usfirst.frc3620.logger.IFastDataLogger;

import edu.wpi.first.wpilibj.RobotController;

import java.util.Date;

public class ShootingDataLogger {
    public static IFastDataLogger getShootingDataLogger (String name) {
        return getShootingDataLogger(name, 15.0);
    }

    public static IFastDataLogger getShootingDataLogger (String name, double length) {
        ShooterSubsystem shooterSubsystem = RobotContainer.shooterSubsystem;
        PreShooterSubsystem preShooterSubsystem = RobotContainer.preShooterSubsystem;
        TurretSubsystem turretSubsystem = RobotContainer.turretSubsystem;
        VisionSubsystem visionSubsystem = RobotContainer.visionSubsystem;

        IFastDataLogger dataLogger = new FastDataLoggerCollections();
        dataLogger.setInterval(0.01);
        dataLogger.setMaxLength(length);
        dataLogger.setFilename(name);
        Date timestamp = new Date();
        dataLogger.setFilenameTimestamp(timestamp);

        dataLogger.addMetadata("timestamp", timestamp.toString());

        dataLogger.addDataProvider("battery_voltage", () -> RobotController.getBatteryVoltage());

        addDataProviders(dataLogger, shooterSubsystem.getTopStatus());
        addDataProviders(dataLogger, shooterSubsystem.getBackStatus());
        addDataProviders(dataLogger, preShooterSubsystem.getPreshooterStatus());

        dataLogger.addDataProvider("hood.requested", () -> shooterSubsystem.getRequestedHoodPosition());
        dataLogger.addDataProvider("hood.position", () -> shooterSubsystem.getHoodPosition());

        if (RobotContainer.turretSubsystemturretSpinner != null) {
            dataLogger.addDataProvider("turret.power", () -> RobotContainer.turretSubsystemturretSpinner.getAppliedOutput());
            dataLogger.addDataProvider("turret.current", () -> RobotContainer.turretSubsystemturretSpinner.getOutputCurrent());
        }
        dataLogger.addDataProvider("turret.requested", () -> turretSubsystem.getRequestedTurretPosition());
        dataLogger.addDataProvider("turret.position", () -> turretSubsystem.getCurrentTurretPosition());

        dataLogger.addDataProvider("vision.target.found", () -> visionSubsystem.isTargetFound() ? 1 : 0);
        dataLogger.addDataProvider("vision.target.centered", () -> visionSubsystem.isTargetCentered() ? 1 : 0);
        dataLogger.addDataProvider("vision.target.xposition", () -> visionSubsystem.getTargetXLocation());
        dataLogger.addDataProvider("vision.target.xdegrees", () -> visionSubsystem.getTargetXDegrees());
        dataLogger.addDataProvider("vision.target.yposition", () -> visionSubsystem.getTargetYLocation());
        dataLogger.addDataProvider("vision.target.range", () -> ShooterCalculator.calcDistanceFromHub(visionSubsystem.getTargetYLocation()));

        return dataLogger;
    }

    static void addDataProviders (IFastDataLogger dl, MotorStatus s) {
        String n = s.getName();
        dl.addDataProvider(n + ".velocity.requested", () -> s.getRequestedSensorVelocity());
        dl.addDataProvider(n + ".velocity.actual", () -> s.getActualSensorVelocity());
        dl.addDataProvider(n + ".rpm.requested", () -> s.getRequestedRPM());
        dl.addDataProvider(n + ".rpm.actual", () -> s.getActualRPM());
        dl.addDataProvider(n + ".current.stator", () -> s.getStatorCurrent());
        dl.addDataProvider(n + ".current.supply", () -> s.getSupplyCurrent());
    }

}