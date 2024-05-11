package frc.team4481.lib.filter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * A filter that scales the speeds of the robot.
 */
public class ScaleSpeedsFilter extends FilterDecorator {
    double scale;
    double angularScale;

    /**
     * Constructs a new ScaleSpeedsFilter.
     * Defaults to scale of 1
     * @param filter the filter to apply to the speeds
     */
    public ScaleSpeedsFilter(ChassisSpeedsFilter filter) {
        super(filter);
        scale = 1;
        angularScale = 1;
    }

    /**
     * Constructs a new ScaleSpeedsFilter.
     * @param filter the filter to apply to the speeds
     * @param scale scale to apply to translation
     * @param angularScale scale to apply to rotation
     */
    public ScaleSpeedsFilter(ChassisSpeedsFilter filter, double scale, double angularScale) {
        super(filter);
        this.scale = scale;
        this.angularScale = angularScale;
    }

    /**
     * Returns the filtered speeds.
     * @return the filtered speeds
     */
    public ChassisSpeeds getFilteredSpeeds() {
        ChassisSpeeds speeds = tempFilter.getFilteredSpeeds();

        return new ChassisSpeeds(
                speeds.vxMetersPerSecond * scale,
                speeds.vyMetersPerSecond * scale,
                speeds.omegaRadiansPerSecond * angularScale
        );
    }

    public void setScale(double scale){
        this.scale = scale;
    }

    public void setAngularScale(double angularScale){
        this.angularScale = angularScale;
    }
}
