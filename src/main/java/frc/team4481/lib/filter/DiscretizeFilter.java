package frc.team4481.lib.filter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * A filter that discretizes the chassis speeds.
 * Because the loops are only run at a certain interval,
 * the chassis speeds should be adjusted to this interval.
 * This makes translating and rotating at the same time more accurate
 */
public class DiscretizeFilter extends FilterDecorator {
    double dt;

    /**
     * Constructs a new DiscretizeFilter.
     * @param filter the filter to apply to the speeds
     * @param dt looper time
     */
    public DiscretizeFilter(ChassisSpeedsFilter filter, double dt) {
        super(filter);
        this.dt = dt;
    }

    /**
     * Returns the filtered speeds.
     * @return the filtered speeds
     */
    public ChassisSpeeds getFilteredSpeeds() {
        ChassisSpeeds speeds = tempFilter.getFilteredSpeeds();

        return ChassisSpeeds.discretize(speeds, dt);
    }
}
