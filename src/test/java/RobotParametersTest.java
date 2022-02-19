import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.lang.reflect.Type;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.reflect.TypeToken;
import org.junit.Test;
import org.usfirst.frc3620.misc.RobotParameters;
import org.usfirst.frc3620.misc.RobotParametersContainer;
import org.usfirst.frc3620.misc.Minifier;

public class RobotParametersTest {

    static String test_parameters_filename = "src/test/java/RobotParametersTest.json";
    static Gson gson = new GsonBuilder().setPrettyPrinting().serializeNulls().create();

    public static class TestRobotParameters extends RobotParameters {
        Double offset;
        public TestRobotParameters() {
            super();
        }

        @Override
        public String toString() {
            return "MyConfig [" + macAddress + "," + makeAllCANDevices + "," + offset + "]";
        }
    }

    @Test
    public void l1_test01() throws IOException {
        String json = Files.readString(Path.of(test_parameters_filename));
        json = Minifier.minify(json);

        Type listOfMyClassObject2 = new TypeToken<ArrayList<TestRobotParameters>>() {}.getType();
        List<TestRobotParameters> list2 = gson.fromJson(json, listOfMyClassObject2);
        System.out.println (list2);

        Map<String, TestRobotParameters> map2 = RobotParametersContainer.makeParameterMap(list2);
        System.out.println (map2);

        Type listOfMyClassObject3 = new TypeToken<ArrayList<RobotParameters>>() {}.getType();
        List<RobotParameters> list3 = gson.fromJson(json, listOfMyClassObject3);
        System.out.println (list3);

        Map<String, RobotParameters> map3 = RobotParametersContainer.makeParameterMap(list3);
        System.out.println (map3);
    }

    @Test
    public void l1_testMinifier() {
        StringWriter sw = new StringWriter();
        PrintWriter pw = new PrintWriter(sw);
        pw.println ("[");
        pw.println ("   // ignore me");
        pw.println ("   \"a\",");
        pw.println ("   1");
        pw.println ("   /* foo */, 2");
        pw.println ("]");

        String s = sw.toString();
        System.out.println (s);
        String s2 = Minifier.minify(s);
        System.out.println (s2);
    }

    public void l2_class() {
        System.out.println (TestRobotParameters.class.isLocalClass());
        System.out.println (TestRobotParameters.class.isMemberClass());
        System.out.println (TestRobotParameters.class.getEnclosingClass());
        System.out.println (TestRobotParameters.class.getModifiers());
    }
    @Test
    public void l2_testBasic() {


        RobotParameters rb = RobotParametersContainer.getRobotParameters(RobotParameters.class, test_parameters_filename);
        System.out.println (rb);

        TestRobotParameters trb = RobotParametersContainer.getRobotParameters(TestRobotParameters.class, test_parameters_filename);
        System.out.println (trb);
    }
}