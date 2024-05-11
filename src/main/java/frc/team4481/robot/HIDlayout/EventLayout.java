package frc.team4481.robot.HIDlayout;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.team4481.lib.controller.ControlDevice;
import frc.team4481.lib.hid.HIDLayout;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.lib.throwable.HardwareException;
import frc.team4481.robot.configuration.Configuration;
import frc.team4481.robot.configuration.ConfigurationHandler;

public class EventLayout extends HIDLayout {
    private final SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
    private final ConfigurationHandler configHandler = ConfigurationHandler.getInstance();


    public EventLayout(XboxController driver, ControlDevice operator) {
        super(driver, operator);
    }

    @Override
    public void getSubsystemManagers() {

    }

    @Override
    public void updateOrange() throws HardwareException {
        //Update configuration
        Configuration config = configHandler.getConfig();
    }

    @Override
    public void updateBlack() throws HardwareException {
    }


}





