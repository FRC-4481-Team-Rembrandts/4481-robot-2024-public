package frc.team4481.robot.configuration;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.team4481.lib.controller.ControlDevice;
import frc.team4481.lib.hid.HIDLayout;
import frc.team4481.robot.HIDlayout.TestLayout;

public class TestConfig implements Configuration {
    private HIDLayout layout = null;

    @Override
    public double getMaxDriveVelocity() {
        return 6.4;
    }
    @Override
    public double getMaxTurnVelocity() {
        return 4.6;
    }

    @Override
    public boolean isAllowedToDo180() {
        return false;
    }

    @Override
    public HIDLayout getHIDLayout(XboxController driver, ControlDevice operator) {
        if (layout == null){
            layout = new TestLayout(driver, operator);
        }
        return layout;
    }
}

