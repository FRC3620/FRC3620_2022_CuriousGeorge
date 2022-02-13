package org.usfirst.frc3620.misc;

/**
 * Container for RobotParameters; designed to be subclassed.
 */
public class RobotParameters {
    protected String macAddress;
    protected boolean competitionRobot;

    public RobotParameters() {
        macAddress = "";
        competitionRobot = false;
    }

    public String getMacAddress() {
        return macAddress;
    }

    public boolean isCompetitionRobot() {
        return competitionRobot;
    }
}