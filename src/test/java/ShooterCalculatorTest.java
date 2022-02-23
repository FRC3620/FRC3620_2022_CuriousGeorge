import org.junit.Test;
import static org.junit.Assert.assertEquals;

import frc.robot.miscellaneous.ShooterCalculator;

public class ShooterCalculatorTest {

    @Test
    public void checkForZero() {
        double allowedSlop = 0.2;
        double expected = 0.0;
        double calculated = ShooterCalculator.calculateZero(22);
        assertEquals("did not get the expected zero!", expected, calculated, allowedSlop);
    }


    @Test
    public void check1800RPM() {

        double allowdSlop = 0.5;
        double expected = 1600.0;
        double calcuated = ShooterCalculator.calculateBackspinRPM(1800.0);
        assertEquals("did not get the expected result!", expected, calcuated, allowdSlop);

    }
}
