import org.junit.Test;
import org.usfirst.frc3620.logger.EventLogging;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** Add your docs here. */
public class TracebackTest {
    @Test
    public void t1() {
        called();
        intermediate();
    }

    void intermediate() {
        called();
    }

    void called() {
        String s = EventLogging.myAndCallersNames();
        System.out.println(s);
    }
}
