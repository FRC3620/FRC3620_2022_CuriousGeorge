import java.io.PrintWriter;
import java.io.StringWriter;
import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.reflect.TypeToken;
import org.junit.Test;
import org.usfirst.frc3620.misc.ConfigurationItem;
import org.usfirst.frc3620.misc.Minifier;

public class ConfigurationTest {

    class MyConfig extends ConfigurationItem {
        double offset;
        MyConfig (String _m, boolean _c, double _o) {
            super(_m, _c);
            offset = _o;
        }

        @Override
        public String toString() {
            return "MyConfig [" + macAddress + "," + competitionRobot + "," + offset + "]";
        }
    }

    @Test
    public void test01() {
        List<MyConfig> list1 = new ArrayList<>();
        list1.add(new MyConfig("aa:bb:cc:dd:ee:ff", true, 4.4));
        list1.add(new MyConfig("00:11:22:33:44:55", false, 5.5));

        Gson gson = new GsonBuilder().setPrettyPrinting().create();

        String json = gson.toJson(list1);

        System.out.println (json);

        Type listOfMyClassObject2 = new TypeToken<ArrayList<MyConfig>>() {}.getType();
        List<MyConfig> list2 = gson.fromJson(json, listOfMyClassObject2);
        System.out.println (list2);

        Map<String,MyConfig> map2 = ConfigurationItem.makeConfigurationMap(list2);
        System.out.println (map2);

        Type listOfMyClassObject3 = new TypeToken<ArrayList<ConfigurationItem>>() {}.getType();
        List<ConfigurationItem> list3 = gson.fromJson(json, listOfMyClassObject3);
        System.out.println (list3);

        Map<String,ConfigurationItem> map3 = ConfigurationItem.makeConfigurationMap(list3);
        System.out.println (map3);

    }

    @Test
    public void testMinifier() {
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

}