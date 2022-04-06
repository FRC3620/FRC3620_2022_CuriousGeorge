import org.junit.Test;

/* make a test that does nothing so we never have trouble with Gradle test not finding any tests */
public class DummyTest {
    @Test
    public void doNothing() {
    }

    @Test
    public void nanTest() {
        double x = 2.0 * Double.NaN;
        System.out.println(x);
    }

    //@Test
    public void llCalc() {
        double targetOffsetAngle_Vertical = -3.79;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 36.0;

        // distance from the target to the floor
        double goalHeightInches = 105.0;

        // how many degrees back is your limelight rotated from perfectly vertical?
        for (double limelightMountAngleDegrees = 10.0; limelightMountAngleDegrees < 35; limelightMountAngleDegrees += 1.0) {

            double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

            // calculate distance
            double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)
                    / Math.tan(angleToGoalRadians);
            System.out.println("" + limelightMountAngleDegrees + " " + (distanceFromLimelightToGoalInches / 12.0));

        }
    }
}
