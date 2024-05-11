package frc.team4481.lib.filter;


import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * A decorator that applies a set of filters to the speeds of the robot.
 */
public abstract class FilterDecorator implements ChassisSpeedsFilter {
    protected ChassisSpeedsFilter tempFilter;

    /**
     * Constructs a new FilterDecorator.
     * @param chassisSpeedsFilter the filter to apply to the speeds
     */
    public FilterDecorator(ChassisSpeedsFilter chassisSpeedsFilter) {
        tempFilter = chassisSpeedsFilter;
    }

    /**
     * Returns the filtered speeds.
     * @return the filtered speeds
     */
    @Override
    public ChassisSpeeds getFilteredSpeeds() {
        return tempFilter.getFilteredSpeeds();
    }
}
