package frc.team4481.robot.configuration;


import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;

import frc.team4481.lib.controller.ControlDevice;
import frc.team4481.lib.hid.HIDLayout;

/**
 * Interface for the configurations of different subsystems
 */
public interface Configuration {

    //Drivetrain
    double getMaxDriveVelocity();
    double getMaxTurnVelocity();
    boolean isAllowedToDo180();

    //HIDLayout
    HIDLayout getHIDLayout(XboxController driver, ControlDevice operator);
}
