import org.junit.Test;
import static org.junit.Assert.assertEquals;

import frc.robot.subsystems.ShooterSubsystem;

public class HoodCalculatorTest {
   
    @Test
    public void check40Degrees() {
        double allowdSlop = .1;
        double expected = 110.0;
        double calcuated = ShooterSubsystem.calculateHoodRotations(40);
        assertEquals("did not get the expected result!", expected, calcuated, allowdSlop);
    } 

    @Test
    public void check83Degrees() {
        double allowdSlop = .1;
        double expected = 0;
        double calcuated = ShooterSubsystem.calculateHoodRotations(83);
        assertEquals("did not get the expected result!", expected, calcuated, allowdSlop);
    }
}
