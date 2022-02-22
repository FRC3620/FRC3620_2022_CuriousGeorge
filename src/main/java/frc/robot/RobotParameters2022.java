package frc.robot;

import java.util.StringJoiner;

import com.google.gson.annotations.SerializedName;
import org.usfirst.frc3620.misc.RobotParameters;

public class RobotParameters2022 extends RobotParameters {
    @SerializedName("swerve.he.rf")
    private Double rightFrontAbsoluteOffset;

    @SerializedName("swerve.he.lf")
    private Double leftFrontAbsoluteOffset;

    @SerializedName("swerve.he.lb")
    private Double leftBackAbsoluteOffset;

    @SerializedName("swerve.he.rb")
    private Double rightBackAbsoluteOffset;

    @SerializedName("swerve.chassis_width")
    private Double chassisWidth;

    @SerializedName("swerve.chassis_length")
    private Double chassisLength;

    private boolean has_climber;
    private boolean has_intake;
    private boolean has_shooter;
    private boolean has_turret;

    private boolean should_run_compressor;

    public RobotParameters2022() {
        super();
    }

    public Double getRightFrontAbsoluteOffset() {
        return rightFrontAbsoluteOffset;
    }

    public Double getLeftFrontAbsoluteOffset() {
        return leftFrontAbsoluteOffset;
    }

    public Double getRightBackAbsoluteOffset() {
        return rightBackAbsoluteOffset;
    }

    public Double getLeftBackAbsoluteOffset() {
        return leftBackAbsoluteOffset;
    }

    public Double getChassisLength() {
        return chassisLength;
    }

    public Double getChassisWidth() {
        return chassisWidth;
    }

    public boolean hasClimber() {
        return has_climber;
    }

    public boolean hasIntake() {
        return has_intake;
    }

    public boolean hasShooter() {
        return has_shooter;
    }

    public boolean hasTurret() {
        return has_turret;
    }

    public boolean shouldRunCompressor() {
        return should_run_compressor;
    }

    public String whichSwerveParametersAreMissing() {
        StringJoiner l = new StringJoiner("|");
        if (leftFrontAbsoluteOffset == null) l.add("swerve.he.lf");
        if (rightFrontAbsoluteOffset == null) l.add("swerve.he.rf");
        if (leftBackAbsoluteOffset == null) l.add("swerve.he.lb");
        if (rightBackAbsoluteOffset == null) l.add("swerve.he.rb");
        if (chassisLength == null) l.add("swerve.chassis_length");
        if (chassisWidth == null) l.add("swerve.chassis_width");
        if (l.length() == 0) return null;
        return l.toString();
    }

    @Override
    public String toString() {
        return new StringJoiner(", ", RobotParameters2022.class.getSimpleName() + "[", "]")
                .add("name='" + name + "'")
                .add("macAddress='" + macAddress + "'")
                .add("makeAllCANDevices=" + makeAllCANDevices)
                .add("has_climber=" + has_climber)
                .add("has_intake=" + has_intake)
                .add("has_shooter=" + has_shooter)
                .add("has_turret=" + has_turret)
                .add("should_run_compressor=" + should_run_compressor)
                .add("rightFrontAbsoluteOffset=" + rightFrontAbsoluteOffset)
                .add("leftFrontAbsoluteOffset=" + leftFrontAbsoluteOffset)
                .add("leftBackAbsoluteOffset=" + leftBackAbsoluteOffset)
                .add("rightBackAbsoluteOffset=" + rightBackAbsoluteOffset)
                .add("chassisWidth=" + chassisWidth)
                .add("chassisLength=" + chassisLength)
                .toString();
    }
}