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

    //@Test
    public void silly() {
        for (double i = 6.0; i < 30.0; i += 0.1) {
            double main_rpm = ShooterCalculator.calcMainRPM(i);
            double back_rpm = ShooterCalculator.calculateBackspinRPM(main_rpm);
            double hood = ShooterCalculator.calcHoodPosition(i);

            System.out.println (i + " " + main_rpm + " " + back_rpm + " " + hood);
        }
    }


}
