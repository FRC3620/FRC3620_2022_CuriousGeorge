package org.usfirst.frc3620.misc;

/**
 * Container for RobotParameters; designed to be subclassed.
 */
public class RobotParameters {
    protected String macAddress;
    protected boolean makeAllCANDevices;
    protected String name;

    public RobotParameters() {
        macAddress = "";
        makeAllCANDevices = false;
    }

    public String getMacAddress() {
        return macAddress;
    }

    public boolean shouldMakeAllCANDevices() {
        return makeAllCANDevices;
    }

    public String getName() {
        return name;
    }

    @Override
    public String toString() {
        return "RobotParameters [" + name + "," + macAddress + "," + makeAllCANDevices + "]";
    }

}