package frc.robot;

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

    private String name;

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

    public String getName() {
        return name;
    }

    @Override
    public String toString() {
        final StringBuilder sb = new StringBuilder("RobotParameters2022{");
        sb.append("macAddress='").append(macAddress).append('\'');
        sb.append(", competitionRobot=").append(competitionRobot);
        sb.append(", name='").append(name).append('\'');
        sb.append(", rightFrontAbsoluteOffset=").append(rightFrontAbsoluteOffset);
        sb.append(", leftFrontAbsoluteOffset=").append(leftFrontAbsoluteOffset);
        sb.append(", leftBackAbsoluteOffset=").append(leftBackAbsoluteOffset);
        sb.append(", rightBackAbsoluteOffset=").append(rightBackAbsoluteOffset);
        sb.append('}');
        return sb.toString();
    }

}
