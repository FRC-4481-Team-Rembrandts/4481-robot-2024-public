package frc.team4481.lib.filter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.opencv.core.Mat;

/**
 * A filter that scales the speeds of the robot.
 */
public class DeadZoneFilter extends FilterDecorator {
    double deadzone = 0;

    /**
     * Constructs a new DeadZoneFilter.
     * Defaults to a deadzone of 0
     * @param filter the filter to apply to the speeds
     */
    public DeadZoneFilter(ChassisSpeedsFilter filter) {
        super(filter);
        deadzone = 0;
    }

    /**
     * Constructs a new DeadZoneFilter.
     * @param filter the filter to apply to the speeds
     * @param deadzone deadzone for the controller
     */
    public DeadZoneFilter(ChassisSpeedsFilter filter, double deadzone) {
        super(filter);
        this.deadzone = deadzone;
    }

    /**
     * Returns the deadzone speeds.
     * @return the deadzone speeds
     */
    public ChassisSpeeds getFilteredSpeeds() {
        ChassisSpeeds speeds = tempFilter.getFilteredSpeeds();
            if (Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2) <= Math.pow(deadzone, 2)){
                speeds.vyMetersPerSecond = 0;
                speeds.vxMetersPerSecond = 0;
            }
            if(Math.abs(speeds.omegaRadiansPerSecond) < deadzone){
                speeds.omegaRadiansPerSecond = 0;
            }
        return new ChassisSpeeds(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond
        );
    }

    /**
     * Sets the deadzone for the controller
     * @param deadzone the deadzone for the controller
     */
    public void setDeadzone(double deadzone){
        this.deadzone = deadzone;
    }

}
