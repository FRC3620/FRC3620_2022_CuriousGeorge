package frc.robot;

import java.util.ArrayList;
import java.util.List;

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

    public String whichSwerveParametersAreMissing() {
        List<String> l = new ArrayList<>();
        if (leftFrontAbsoluteOffset == null) l.add("swerve.he.lf");
        if (rightFrontAbsoluteOffset == null) l.add("swerve.he.rf");
        if (leftBackAbsoluteOffset == null) l.add("swerve.he.lb");
        if (rightBackAbsoluteOffset == null) l.add("swerve.he.rb");
        if (chassisLength == null) l.add("swerve.chassis_length");
        if (chassisWidth == null) l.add("swerve.chassis_width");
        if (l.size() == 0) return null;
        return String.join("|", l);
    }

    @Override
    public String toString() {
        final StringBuilder sb = new StringBuilder("RobotParameters2022{");
        sb.append("macAddress='").append(macAddress).append('\'');
        sb.append(", makeAllCANDevices=").append(makeAllCANDevices);
        sb.append(", name='").append(name).append('\'');
        sb.append(", rightFrontAbsoluteOffset=").append(rightFrontAbsoluteOffset);
        sb.append(", leftFrontAbsoluteOffset=").append(leftFrontAbsoluteOffset);
        sb.append(", leftBackAbsoluteOffset=").append(leftBackAbsoluteOffset);
        sb.append(", rightBackAbsoluteOffset=").append(rightBackAbsoluteOffset);
        sb.append(", chassisWidth=").append(chassisWidth);
        sb.append(", chassisLength=").append(chassisLength);
        sb.append('}');
        return sb.toString();
    }

}