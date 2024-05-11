package frc.team4481.lib.controller;

import frc.team4481.lib.throwable.HardwareException;

public interface IPS4HID {
    /**
     * The naming of the button id configuration
     */
    enum Button{
        CROSS(1),
        CIRCLE(2),
        SQUARE(3),
        TRIANGLE(4),

        BUMPER_L1(5),
        BUMPER_R1(6),

        SHARE(7),
        OPTIONS(8),

        LEFTSTICK_BUTTON(9),
        RIGHTSTICK_BUTTON(10);


        public final int id;

        Button(int id) {
            this.id = id;
        }
    }

    enum DpadButton{
        DPAD_N(0),
        DPAD_NE(45),
        DPAD_E(90),
        DPAD_SE(135),
        DPAD_S(180),
        DPAD_SW(225),
        DPAD_W(270),
        DPAD_NW(315);
        public final int id;
        DpadButton(int id) {
            this.id = id;
        }
    }
    /**
     * The naming of the axis id configuration
     */
    enum Axis{
        LEFTSTICK_X(0),
        LEFTSTICK_Y(1),
        TRIGGER_L2(2),
        TRIGGER_R2(3),
        RIGHTSTICK_X(4),
        RIGHTSTICK_Y(5);

        public final int id;

        Axis(int id) {
            this.id = id;
        }
    }
    //standard SubsystemController functions

    /**
     *
     * @param button The requested button
     * @return boolean (true = button pressed, false = button not pressed)
     * @throws HardwareException an exception that can be thrown if the button is broken
     */
    boolean getButtonValue(Button button) throws HardwareException;

    /**
     *
     * @param axis The requested axis
     * @return double value (-1.00 -- 1.00)
     */
    double getAxisValue(Axis axis);
    /**
     *
     * @param button The orientation of the requested DPadButton
     * @return if the requested button is pressed.
     */
    boolean getDpadValue(DpadButton button);
}
