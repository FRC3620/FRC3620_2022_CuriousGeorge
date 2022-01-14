// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc3620.misc;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Add your docs here. */
public class ConfigurationItem {
    protected String macAddress;
    protected boolean competitionRobot;

    protected ConfigurationItem (String _macAddress, boolean _competitionRobot) {
        this.macAddress = _macAddress;
        this.competitionRobot = _competitionRobot;
    }

    public String getMacAddress() {
        return macAddress;
    }

    public boolean isCompetitionRobot() {
        return competitionRobot;
    }

    public static <T extends ConfigurationItem> Map<String,T> makeConfigurationMap (List<T> l) {
        Map<String,T> rv = new HashMap<>();
        for (T c : l) {
            rv.put(c.macAddress, c);
        }
        return rv;
    }
}