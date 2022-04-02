import org.junit.Test;
import static org.junit.Assert.assertEquals;

import frc.robot.subsystems.ShooterSubsystem;

public class HoodCalculatorTest {
   
    @Test
    public void check40Degrees() {
        double allowdSlop = .1;
        double expected = 28.976;
        double calculated = ShooterSubsystem.calculateHoodRotations(40);
        System.out.println("40 degree calculation: " + calculated);
        assertEquals("did not get the expected result!", expected, calculated, allowdSlop);
    } 

    @Test
    public void check80Degrees() {
        double allowdSlop = .1;
        double expected = 1.6665;
        double calculated = ShooterSubsystem.calculateHoodRotations(80);
        System.out.println("80 degree calculation: " + calculated);
        assertEquals("did not get the expected result!", expected, calculated, allowdSlop);
    }
}
