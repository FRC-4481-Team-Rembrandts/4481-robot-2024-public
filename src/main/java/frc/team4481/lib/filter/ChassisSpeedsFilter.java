package frc.team4481.lib.filter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * A filter is applied to the ChassisSpeeds of the robot.
 */
public interface ChassisSpeedsFilter {

    /**
     * Returns the filtered speeds.
     * @return the filtered speeds
     */
    ChassisSpeeds getFilteredSpeeds();
}
