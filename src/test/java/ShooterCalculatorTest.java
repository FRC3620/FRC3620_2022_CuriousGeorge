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
}
