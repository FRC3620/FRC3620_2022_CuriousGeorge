import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import frc.robot.RobotParameters2022;
import org.junit.Test;
import org.usfirst.frc3620.misc.RobotParametersContainer;

public class RobotParameters2022Test {

    static Gson gson = new GsonBuilder().setPrettyPrinting().serializeNulls().create();

    static String filename = "src/main/deploy/robot_parameters.json";

    @Test
    public void l0_testBasic() {
        RobotParameters2022 rb = RobotParametersContainer.getRobotParameters(RobotParameters2022.class, filename, "00-80-2F-18-5C-5F");
        System.out.println (rb);
        rb = RobotParametersContainer.getRobotParameters(RobotParameters2022.class, filename, "00:11:22:33:44:55");
        System.out.println (rb);
        rb = RobotParametersContainer.getRobotParameters(RobotParameters2022.class, filename, "foo");
        System.out.println (rb);
        rb = RobotParametersContainer.getRobotParameters(RobotParameters2022.class, null, "foo");
        System.out.println (rb);
    }
}