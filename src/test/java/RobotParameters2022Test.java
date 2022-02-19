import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;

import java.nio.file.Path;
import java.nio.file.Paths;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import frc.robot.RobotParameters2022;
import org.junit.Test;
import org.usfirst.frc3620.misc.RobotParametersContainer;

public class RobotParameters2022Test {

    static Gson gson = new GsonBuilder().setPrettyPrinting().serializeNulls().create();

    static Path path = Paths.get("src/main/deploy/robot_parameters.json");

    @Test
    public void makeSureRobotParametersFileIsGood() {
        RobotParameters2022 rb;

        rb = RobotParametersContainer.getRobotParameters(RobotParameters2022.class, path, "00-80-2F-17-EB-08");
        assertNull(path.toString() + " is missing swerve parameters for 00-80-2F-17-EB-08", rb.whichSwerveParametersAreMissing());
        
        rb = RobotParametersContainer.getRobotParameters(RobotParameters2022.class, path, "00-80-2F-18-5C-5F");
        assertNull(path.toString() + " is missing swerve parameters for 00-80-2F-18-5C-5F", rb.whichSwerveParametersAreMissing());
        
        rb = RobotParametersContainer.getRobotParameters(RobotParameters2022.class, path, "00-11-22-33-44-55");
        assertNotNull(path.toString() + " should be missing swerve parameters for 00-11-22-33-44-55", rb.whichSwerveParametersAreMissing());
    }
}