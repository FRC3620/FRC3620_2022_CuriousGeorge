import java.util.HashMap;
import java.util.Map;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import org.junit.Test;

public class GsonTest {

    @Test
    public void nan() {
        Map<String,Object> m = new HashMap<>();
        m.put("a", Double.NaN);
        System.out.println(m);

        Gson gson = new GsonBuilder().serializeSpecialFloatingPointValues().create();
        String j = gson.toJson(m);
        System.out.println(j);
    }
}
