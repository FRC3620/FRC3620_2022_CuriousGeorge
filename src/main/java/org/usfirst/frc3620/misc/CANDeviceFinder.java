package org.usfirst.frc3620.misc;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.*;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.hal.can.CANJNI;

public class CANDeviceFinder {
    static Logger logger = EventLogging.getLogger(CANDeviceFinder.class, Level.INFO);

    Set<CANDeviceId> deviceSet = new TreeSet<>();

    /*
     * this is a map, keyed by CANDeviceType, whose values are sets contained
     * the device numbers for all the present devices of that type.
     */
    Map<CANDeviceType, Set<Integer>> byDeviceType = new TreeMap<>();

    public CANDeviceFinder() {
        super();
        find();
        // research();
    }

    public boolean isDevicePresent(CANDeviceType deviceType, int id) {
        return isDevicePresent(deviceType, id, null);
    }

    public boolean isDevicePresent(CANDeviceType deviceType, int id, String whatItIs) {
        boolean rv = false;
        Set<Integer> deviceTypeSet = byDeviceType.get(deviceType);
        if (deviceTypeSet != null) {
            rv = deviceTypeSet.contains(id);
        }
        if (!rv) {
            if (whatItIs == null) {
                logger.warn("{} {} is missing from the CAN bus!!!", deviceType, id);
            } else {
                logger.warn("{} {} ({}) is missing from the CAN bus!!!", deviceType, id, whatItIs);
            }
        }
        return rv;
    }

    public boolean isPowerDistributionPresent() {
        return isDevicePresent(CANDeviceType.CTRE_PDP, 0);
    }

    /**
     * 
     * @return ArrayList of strings holding the names of devices we've found.
     */
    public Set<CANDeviceId> getDeviceSet() {
        return deviceSet;
    }

    abstract class CanFinder {
        int[] ids;
        long[] ts0;
        Set<Integer> idsPresent = new TreeSet<>();

        void pass1() {
            ts0 = new long[ids.length];
            for (int i = 0; i < ids.length; ++i) {
                ts0[i] = checkMessage(ids[i]);
                //logger.info ("pass1 looking for {} got {}", String.format("%08x", ids[i]), ts0[i]);
            }
        }

        void pass2() {
            long[] ts1 = new long[ids.length];
            for (int i = 0; i < ids.length; ++i) {
                ts1[i] = checkMessage(ids[i]);
                //logger.info ("pass2 looking for {} got {}", String.format("%08x", ids[i]), ts1[i]);
            }
            for (int i = 0; i < ids.length; ++i) {
                if (ts0[i] >= 0 && ts1[i] >= 0 && ts0[i] != ts1[i]) {
                    // logger.info ("found {}", String.format("%08x", ids[i]));
                    idsPresent.add(ids[i]);
                }
            }
        }

        abstract void report();
    }

    class DeviceFinder extends CanFinder {
        Set<CANDeviceId> deviceSet;
        CANDeviceType canDeviceType;
        DeviceFinder(CANDeviceType canDeviceType, Set<CANDeviceId> deviceSet) {
            super();

            this.deviceSet = deviceSet;
            this.canDeviceType = canDeviceType;

            ids = new int[canDeviceType.getMaxDevices()];
            for (int i = 0; i < canDeviceType.getMaxDevices(); i++) {
                ids[i] = (canDeviceType.getMsgId() & 0xffffffc0) | (i & 0x3f);
            }
        }

        @Override
        void report() {
            for (int id: idsPresent) {
                int deviceId = extractDeviceId(id);
                deviceSet.add(new CANDeviceId(canDeviceType, deviceId));//NOPMD
            }
        }
    }

    class APIFinder extends CanFinder {
        CANDeviceType canDeviceType;
        APIFinder(CANDeviceType canDeviceType, int deviceId) {
            super();
            this.canDeviceType = canDeviceType;

            ids = new int[1024];
            for (int i = 0; i < 1024; i++) {
                ids[i] = (canDeviceType.msgId & 0xffff0000) | ((i << 6) & 0xffc0) | (deviceId & 0x3f);
            }
        }

        @Override
        void report() {
            for (int id: idsPresent) {
                int apiId = extractApiId(id);
                logger.info ("{}: API {} {} = msg {}", canDeviceType, String.format("%2d", apiId), String.format("0x%03X", apiId), String.format("%08X", id));
            }
        }
    }
    
    /**
     * polls for received framing to determine if a device is deviceSet. This is
     * meant to be used once initially (and not periodically) since this steals
     * cached messages from the robot API.
     */
    public void find() {
        deviceSet.clear();
        byDeviceType.clear();

        logger.info ("calling find()");
        List<CanFinder> finders = new ArrayList<>();

        for (CANDeviceType canDeviceType : CANDeviceType.values()) {
            finders.add(new DeviceFinder(canDeviceType, deviceSet));
        }

        findDetails(finders);
    }

    public void research() {
        logger.info ("calling research()");
        List<CanFinder> finders = new ArrayList<>();
        
        finders.add(new APIFinder(CANDeviceType.CTRE_PCM, 0)); // PCM 0
        finders.add(new APIFinder(CANDeviceType.CTRE_PDP, 0)); // PDP 0
        finders.add(new APIFinder(CANDeviceType.REV_PH, 1)); // PCM 0
        finders.add(new APIFinder(CANDeviceType.REV_PDH, 1)); // PDP 0
        finders.add(new APIFinder(CANDeviceType.TALON, 1)); // SRX #1
        finders.add(new APIFinder(CANDeviceType.VICTOR_SPX, 2)); // SPX #2

        findDetails(finders);
    }

    void findDetails(List<CanFinder> finders) {
        for (CanFinder finder: finders) {
            finder.pass1();
        }

        /* wait 200ms */
        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace(); // NOPMD
        }

        for (CanFinder finder: finders) {
            finder.pass2();
        }

        for (CanFinder finder: finders) {
            finder.report();
        }

        /*
         fill in the byDeviceType map.
         */
        for (CANDeviceId canDeviceId: deviceSet) {
            CANDeviceType canDeviceType = canDeviceId.getDeviceType();
            Set<Integer> deviceNumberSet = byDeviceType.get(canDeviceType);
            if (deviceNumberSet == null) {
                deviceNumberSet = new TreeSet<>();//NOPMD
                byDeviceType.put(canDeviceType, deviceNumberSet);
            }
            deviceNumberSet.add(canDeviceId.getDeviceNumber());
        }
    }
    
    private int extractDeviceId (int canId) {
        return canId & 0x3f;
    }

    private int extractApiId (int canId) {
        return (canId & 0xffc0) >> 6;
    }

    /** helper routine to get last received message for a given ID */
    private ByteBuffer targetID = ByteBuffer.allocateDirect(4);
    private ByteBuffer timeStamp = ByteBuffer.allocateDirect(4);
    private long checkMessage(int id) {
        try {
            targetID.clear();
            targetID.order(ByteOrder.LITTLE_ENDIAN);
            targetID.asIntBuffer().put(0, id);

            timeStamp.clear();
            timeStamp.order(ByteOrder.LITTLE_ENDIAN);
            timeStamp.asIntBuffer().put(0, 0x00000000);

            CANJNI.FRCNetCommCANSessionMuxReceiveMessage(
                    targetID.asIntBuffer(), 0x1fffffff, timeStamp);

            long retval = timeStamp.getInt();
            retval &= 0xFFFFFFFF; /* undo sign-extension */
            return retval;
        } catch (Exception e) {
            return -1;
        }
    }
}