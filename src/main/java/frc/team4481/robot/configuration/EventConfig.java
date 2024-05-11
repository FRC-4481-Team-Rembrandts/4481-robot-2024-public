package frc.team4481.robot.configuration;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.team4481.lib.controller.ControlDevice;
import frc.team4481.lib.hid.HIDLayout;
import frc.team4481.robot.HIDlayout.EventLayout;

public class EventConfig implements Configuration {
    private HIDLayout layout = null;

    @Override
    public double getMaxDriveVelocity() {
        return 2;
    }
    @Override
    public double getMaxTurnVelocity() {
        return 1.5;
    }
    @Override
    public boolean isAllowedToDo180() {
        return false;
    }

    @Override
    public HIDLayout getHIDLayout(XboxController driver, ControlDevice operator) {
        if (layout == null){
            layout = new EventLayout(driver, operator);
        }
        return layout;
    }
}
