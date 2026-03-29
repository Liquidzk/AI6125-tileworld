package tileworld;

/**
 * Parameters
 *
 * @author michaellees
 * Created: Apr 21, 2010
 *
 * Copyright michaellees 
 *
 * Description:
 *
 * Class used to store global simulation parameters.
 * Environment related parameters are still in the TWEnvironment class.
 *
 */
public class Parameters {

    private static final String PROFILE = System.getProperty("tileworld.profile", "config1").trim().toLowerCase();
    private static final boolean CONFIG_TWO = "config2".equals(PROFILE);
    //Simulation Parameters
    public static final int seed = intProperty("tileworld.seed", 4162012); //no effect with gui
    public static final long endTime = longProperty("tileworld.endTime", 5000); //no effect with gui

    //Agent Parameters
    public static final int agentCount = intProperty("tileworld.agentCount", 6);
    public static final int defaultFuelLevel = intProperty("tileworld.defaultFuelLevel", 500);
    public static final int defaultSensorRange = intProperty("tileworld.defaultSensorRange", 3);

    //Environment Parameters
    public static final int xDimension = intProperty("tileworld.xDimension", CONFIG_TWO ? 80 : 50); //size in cells
    public static final int yDimension = intProperty("tileworld.yDimension", CONFIG_TWO ? 80 : 50);

    //Object Parameters
    // mean, dev: control the number of objects to be created in every time step (i.e. average object creation rate)
    public static final double tileMean = doubleProperty("tileworld.tileMean", CONFIG_TWO ? 2.0 : 0.2);
    public static final double holeMean = doubleProperty("tileworld.holeMean", CONFIG_TWO ? 2.0 : 0.2);
    public static final double obstacleMean = doubleProperty("tileworld.obstacleMean", CONFIG_TWO ? 2.0 : 0.2);
    public static final double tileDev = doubleProperty("tileworld.tileDev", CONFIG_TWO ? 0.5 : 0.05);
    public static final double holeDev = doubleProperty("tileworld.holeDev", CONFIG_TWO ? 0.5 : 0.05);
    public static final double obstacleDev = doubleProperty("tileworld.obstacleDev", CONFIG_TWO ? 0.5 : 0.05);
    // the life time of each object
    public static final int lifeTime = intProperty("tileworld.lifeTime", CONFIG_TWO ? 30 : 100);

    private static int intProperty(String key, int defaultValue) {
        return Integer.getInteger(key, defaultValue);
    }

    private static long longProperty(String key, long defaultValue) {
        String value = System.getProperty(key);
        return value == null ? defaultValue : Long.parseLong(value);
    }

    private static double doubleProperty(String key, double defaultValue) {
        String value = System.getProperty(key);
        return value == null ? defaultValue : Double.parseDouble(value);
    }
}
