import org.junit.Test;
import static org.junit.Assert.assertEquals;

import frc.robot.subsystems.ShooterSubsystem;

public class HoodCalculatorTest {
   
    @Test
    public void check40Degrees() {
        double allowdSlop = .1;
        double expected = 30.456;
        double calculated = ShooterSubsystem.calculateHoodRotations(40);
        System.out.println("40 degree calculation: " + calculated);
        assertEquals("did not get the expected result!", expected, calculated, allowdSlop);
    } 

    @Test
    public void check80Degrees() {
        double allowdSlop = .1;
        double expected = 2.096;
        double calculated = ShooterSubsystem.calculateHoodRotations(80);
        System.out.println("80 degree calculation: " + calculated);
        assertEquals("did not get the expected result!", expected, calculated, allowdSlop);
    }

    @Test
    public void hoodAngleTest() {
        double a = ShooterSubsystem.calculateHoodAngle(ShooterSubsystem.calculateHoodRotations(45));
        assertEquals("did not get the expected result!", 45, a, 0.1);
    }
}
