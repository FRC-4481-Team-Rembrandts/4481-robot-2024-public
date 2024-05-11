package frc.team4481.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.team4481.lib.controller.ControlDevice;
import frc.team4481.lib.hid.HIDLayout;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.lib.throwable.HardwareException;
import frc.team4481.robot.configuration.Configuration;
import frc.team4481.robot.configuration.ConfigurationHandler;


public class HIDManager {
    private final ConfigurationHandler configHandler = ConfigurationHandler.getInstance();
    private Configuration config;
    private XboxController mDriver;
    private ControlDevice mOperator;
    private HIDLayout layout;

    public static HIDManager mInstance = null;
    private HIDManager() {
        mDriver = new XboxController(0);
        mOperator = new ControlDevice(1);
    }

    public static HIDManager getInstance() {
        if (mInstance == null) {
            mInstance = new HIDManager();
        }
        return mInstance;
    }

    public void getSubsystemManagers(SubsystemHandler subsystemHandler) {
        configHandler.getLayoutSubsystemManagers(mDriver, mOperator);
    }

    public void update() {
        try {
            config = configHandler.getConfig();
            layout = config.getHIDLayout(mDriver, mOperator);
            layout.updateOrange();
            layout.updateBlack();
        } catch (HardwareException e) {
            e.printStackTrace();
        }
    }
}
