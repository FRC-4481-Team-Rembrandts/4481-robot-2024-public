package frc.team4481.lib.controller;

import edu.wpi.first.wpilibj.Joystick;
import frc.team4481.lib.throwable.HardwareException;

@Deprecated
public class BlackHID extends Joystick implements IPS4HID {

    /**
     * Construct an instance of a joystick. The joystick index is the USB port on the drivers
     * station.
     *
     * @param pPort The port on the Driver Station that the joystick is plugged into.
     */
    public BlackHID(int pPort) {
        super(pPort);
    }

    /**
     *
     * @param button The name of the button as seen on the controller
     * @return boolean (true = button pressed, false = button not pressed)
     */
    @Override
    public boolean getButtonValue(Button button) throws HardwareException {
        switch (button) {
            default:
                return getRawButton(button.id);
        }
    }

    /**
     *
     * @param axis The name of the axis as seen on the controller
     * @return double value (-1.00 -- 1.00)
     */
    @Override
    public double getAxisValue(Axis axis) {
        return getRawAxis(axis.id);
    }
    
    /**
     *
     * @param button The orientation of the requested DPadButton
     * @return if the requested button is pressed.
     */
    @Override
    public boolean getDpadValue(DpadButton button) {return getPOV() == button.id;}
}
