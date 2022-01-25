// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc3620.misc;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

/** Add your docs here. */
public class ConfigurationItem {
    public final static Logger logger = EventLogging.getLogger(ConfigurationItem.class, Level.INFO);

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

    
  static String roboRIOMacAddress = null;
   
  public static String identifyRoboRIO() {
    if (roboRIOMacAddress == null) {
      String rv = "";
      try {
        for (Enumeration<NetworkInterface> e = NetworkInterface
            .getNetworkInterfaces(); e.hasMoreElements();) {
          NetworkInterface network = e.nextElement();
          byte[] mac = network.getHardwareAddress();
          if (mac == null) {
            logger.info("found network {}, no MAC", network.getName());
          } else {
            StringBuilder sb = new StringBuilder();
            for (int i = 0; i < mac.length; i++) {
              sb.append(String.format("%02X%s", mac[i],
                  (i < mac.length - 1) ? "-" : ""));
            }
            String macString = sb.toString();
            logger.info("found network {}, MAC address {}", network.getName(), macString);
            if (network.getName().equals("eth0")) {
              rv = macString;
            }
          }
        }
      } catch (SocketException e) {
        e.printStackTrace();
      }
      if (rv.length() == 0) {
        rv = "(unknown)";
      }
      roboRIOMacAddress = rv;
    }
    return roboRIOMacAddress;
  }

}