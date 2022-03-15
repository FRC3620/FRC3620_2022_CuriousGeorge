package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.miscellaneous.ShooterCalculator;
import org.slf4j.Logger;

import java.util.HashMap;

import com.google.gson.Gson;

public class ShooterDecider {
    static Gson gson = new Gson();

    public static class PewPewData extends HashMap<String, Object> {
        public void fillInVisionXDegress(double v) {
            put("vision.xdegrees", v);
        }

        public void fillInVisionYLocation(double v) {
            put("vision.ylocation", v);
        }

        public void fillInVisionDistance(double v) {
            put("vision.range", v);
        }

        public void fillInVisionData() {
            double targetXDegrees = RobotContainer.visionSubsystem.getTargetXDegrees();
            double targetYLocation = RobotContainer.visionSubsystem.getTargetYLocation();
            double targetDistance = ShooterCalculator.calcDistanceFromHub(targetYLocation);
            fillInVisionXDegress(targetXDegrees);
            fillInVisionYLocation(targetYLocation);
            fillInVisionDistance(targetDistance);
        }
    }

    public static boolean isTurretInPosition(PewPewData pewPewData){
        double tra = RobotContainer.turretSubsystem.getCurrentTurretPosition();
        double trs = RobotContainer.turretSubsystem.getRequestedTurretPosition();
        double trerror = Math.abs(trs - tra);

        pewPewData.put("turret.position.error", trerror);
        pewPewData.put("turret.position.requested", trs);
        pewPewData.put("turret.position.actual", tra);

        SmartDashboard.putNumber("shooter.error.turret", trerror);
        boolean answer = false;
        if (trerror < 2) {
            answer = true;
        }
        SmartDashboard.putBoolean("shooter.ready.turret", answer);
        return answer;
    }

    public static boolean isShooterUpToSpeed(PewPewData pewPewData){
        double ta = RobotContainer.shooterSubsystem.getActualMainShooterVelocity();
        double ts = RobotContainer.shooterSubsystem.getRequestedMainShooterVelocity();
        double terror = Double.NaN;
        if (ts != 0.0) {
            terror = ta / ts;
        }

        pewPewData.put("shooter.speed.ratio", terror);
        pewPewData.put("shooter.speed.requested", ts);
        pewPewData.put("shooter.speed.actual", ta);

        SmartDashboard.putNumber("shooter.error.shooter", terror);
        boolean answer = false;
        if (terror >= 0.98 && terror <= 1.02) {
            answer = true;
        }
        SmartDashboard.putBoolean("shooter.ready.shooter", answer);
        return answer;
    }

    public static boolean isHoodInPosition(PewPewData pewPewData){
        double ha = RobotContainer.shooterSubsystem.getHoodPosition();
        double hs = RobotContainer.shooterSubsystem.getRequestedHoodPosition();
        double herror = Double.NaN;
        if (hs != 0.0) {
            herror = ha / hs;
        }

        pewPewData.put("hood.position.ratio", herror);
        pewPewData.put("hood.position.requested", hs);
        pewPewData.put("hood.position.actual", ha);

        SmartDashboard.putNumber("shooter.error.hood", herror);
        boolean answer = false;
        if (herror >= 0.98 && herror <= 1.02) {
            answer = true;
        }
        SmartDashboard.putBoolean("shooter.ready.hood", answer);
        return answer;
    }

    public static void logPewPewData(Logger logger, String prefix, PewPewData pewPewData) {
        String json = gson.toJson(pewPewData);
        logger.info("{} pewpew: {}", prefix, json);
    }

    public static void showReady(boolean ready) {
        SmartDashboard.putBoolean("shooter.ready", ready);
    }

    public static void showNotReady() {
        SmartDashboard.putBoolean("shooter.ready.shooter", false);
        SmartDashboard.putBoolean("shooter.ready.hood", false);
        SmartDashboard.putBoolean("shooter.ready.turret", false);
        SmartDashboard.putBoolean("shooter.ready", false);
    }

}