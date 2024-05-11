package frc.team4481.lib.hid;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;

import frc.team4481.lib.controller.ControlDevice;
import frc.team4481.lib.throwable.HardwareException;

public abstract class HIDLayout {
    protected ControlDevice operator;
    protected XboxController driver;

    public HIDLayout(XboxController driver, ControlDevice operator){
        this.operator = operator;
        this.driver = driver;
    }

    public abstract void getSubsystemManagers();

    public abstract void updateOrange() throws HardwareException;
    public abstract void updateBlack() throws HardwareException;
}
