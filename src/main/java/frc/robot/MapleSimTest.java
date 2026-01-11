package frc.robot;

public class MapleSimTest {
    private static MapleSimTest instance;


    public static MapleSimTest getInstance() {
        if (instance == null) instance = new MapleSimTest();
        return instance;
    }
}
