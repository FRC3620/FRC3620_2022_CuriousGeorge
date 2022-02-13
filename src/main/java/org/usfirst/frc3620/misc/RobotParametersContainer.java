package org.usfirst.frc3620.misc;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.reflect.TypeToken;
import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;

import java.io.IOException;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Modifier;
import java.lang.reflect.Type;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class RobotParametersContainer {
    public final static Logger logger = EventLogging.getLogger(RobotParameters.class, EventLogging.Level.INFO);

    static Gson gson = new GsonBuilder().setPrettyPrinting().serializeNulls().create();

    static Map<String, ? extends RobotParameters> parameterMap = null;

    public static <T extends RobotParameters> Map<String, T> makeParameterMap(List<T> l) {
        Map<String, T> rv = new HashMap<>();
        for (T c : l) {
            rv.put(c.macAddress, c);
        }
        return rv;
    }

    static <T extends RobotParameters> void readConfiguration(Class parametersClass, Path path) throws IOException {
        if (path == null) {
            path = Path.of("robot_config.json");
        }
        String json = Files.readString(path);
        json = Minifier.minify(json);
        // https://stackoverflow.com/a/52296997
        Type parameterListType = TypeToken.getParameterized(List.class, parametersClass).getType();
        List<T> list = gson.fromJson(json, parameterListType);

        parameterMap = makeParameterMap(list);
    }

    public static RobotParameters getRobotParameters (Class parametersClass, String filename) {
        return getRobotParameters(parametersClass, filename, identifyRoboRIO());
    }

    public static RobotParameters getRobotParameters (Class parametersClass) {
        return getRobotParameters(parametersClass, null, identifyRoboRIO());
    }

    public static RobotParameters getRobotParameters (Class parametersClass, String filename, String mac) {
        if (! RobotParameters.class.isAssignableFrom(parametersClass)) {
            logger.error("getRobotParameters needs a subclass of RobotParameters, returning null");
            return null;
        }
        RobotParameters rv = null;

        logger.info("reading robotParameters from {}", filename);
        if (parameterMap == null) {
            if (filename == null) {
                filename = "robot_config.json";
            }
            try {
                readConfiguration(parametersClass, Path.of(filename));
            } catch (IOException e) {
                logger.error ("can't read configuration at {}", filename);
                logger.error ("caused by", e);
            }
        }
        if (parameterMap != null) {
            rv = parameterMap.get(mac);
            if (rv == null) {
                logger.info ("no entry in {} for \"{}\"", filename, mac);
            }
        }
        if (rv == null) {
            if (parametersClass.isMemberClass() && ((parametersClass.getModifiers() & Modifier.STATIC) == 0)) {
                logger.error("Cannot make a default value; this is a non-static inner class");
            } else {
                try {
                    logger.info("making default {}", parametersClass.getName());
                    Constructor<? extends RobotParameters> c = parametersClass.getConstructor();
                    rv = c.newInstance();
                } catch (InvocationTargetException | InstantiationException | IllegalAccessException | NoSuchMethodException e) {
                    e.printStackTrace(System.err);
                    logger.error("got exception {}, returning null RobotParameters", e);
                }
            }
        }
        if (rv != null) {
            logger.info("robot parameters {}: {}", rv.getClass(), gson.toJson(rv));
        }
        return rv;
    }

    static String roboRIOMacAddress = null;

    public static String identifyRoboRIO() {
        if (roboRIOMacAddress == null) {
            String rv = "";
            try {
                for (Enumeration<NetworkInterface> e = NetworkInterface
                        .getNetworkInterfaces(); e.hasMoreElements(); ) {
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
