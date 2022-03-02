package frc.robot;

import frc.robot.miscellaneous.MotorStatus;
import frc.robot.subsystems.ShooterSubsystem;

import org.usfirst.frc3620.logger.FastDataLoggerCollections;
import org.usfirst.frc3620.logger.IFastDataLogger;

import edu.wpi.first.wpilibj.RobotController;

import java.util.Date;

public class ShootingDataLogger {
    public static IFastDataLogger getShootingDataLogger (String name, ShooterSubsystem shooterSubsystem) {
        return getShootingDataLogger(name, shooterSubsystem, 15.0);
    }

    static void addDataProviders (IFastDataLogger dl, MotorStatus s) {
        String n = s.getName();
        dl.addDataProvider(n + ".velocity.requested", () -> s.getRequestedRPM());
        dl.addDataProvider(n + ".velocity.actual", () -> s.getActualSensorVelocity());
        dl.addDataProvider(n + ".rpm.requested", () -> s.getRequestedRPM());
        dl.addDataProvider(n + ".rpm.actual", () -> s.getActualRPM());
        dl.addDataProvider(n + ".current.stator", () -> s.getStatorCurrent());
        dl.addDataProvider(n + ".current.supply", () -> s.getSupplyCurrent());
    }

    public static IFastDataLogger getShootingDataLogger (String name, ShooterSubsystem shooterSubsystem, double length) {

        IFastDataLogger dataLogger = new FastDataLoggerCollections();
        dataLogger.setInterval(0.005);
        dataLogger.setMaxLength(length);
        dataLogger.setFilename(name);
        Date timestamp = new Date();
        dataLogger.setFilenameTimestamp(timestamp);

        dataLogger.addMetadata("timestamp", timestamp.toString());

        dataLogger.addDataProvider("battery_voltage", () -> RobotController.getBatteryVoltage());

        addDataProviders(dataLogger, shooterSubsystem.getTopStatus());
       

        return dataLogger;
    }
}
