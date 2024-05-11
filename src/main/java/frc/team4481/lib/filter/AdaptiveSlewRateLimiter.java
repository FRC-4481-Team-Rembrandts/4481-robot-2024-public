package frc.team4481.lib.filter;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * A filter that limits the slew rate of the robot to not burn through a set of wheels.
 */
public class AdaptiveSlewRateLimiter extends FilterDecorator {

    private double rateLimit;
    private Rotation2d prevVal;
    private double prevTime;

    private final Rotation2d reverseThreshold;

    /**
     * Constructs a new WeekendSlewRateLimiter.
     * @param filter the filter to apply to the speeds
     * @param reverseThreshold At what angle change the robot should first stop and then reverse direction
     */
    public AdaptiveSlewRateLimiter(ChassisSpeedsFilter filter, Rotation2d reverseThreshold) {
        super(filter);

        rateLimit = 1000; //Default the limit to something very high, this results in almost unlimited direction change
        this.reverseThreshold = reverseThreshold;

        prevVal = new Rotation2d();
        prevTime = MathSharedStore.getTimestamp();
    }

    /**
     * Constructs a new WeekendSlewRateLimiter.
     * @param filter the filter to apply to the speeds
     * @param reverseThreshold At what angle change the robot should first stop and then reverse direction
     * @param rateLimit The rate limit to apply to the direction change
     */
    public AdaptiveSlewRateLimiter(ChassisSpeedsFilter filter, Rotation2d reverseThreshold, double rateLimit) {
        super(filter);

        this.rateLimit = rateLimit;
        this.reverseThreshold = reverseThreshold;

        prevVal = new Rotation2d();
        prevTime = MathSharedStore.getTimestamp();
    }



    /**
     * Returns the filtered speeds.
     * @return the filtered speeds
     */
    public ChassisSpeeds getFilteredSpeeds() {
        ChassisSpeeds speeds = tempFilter.getFilteredSpeeds();

        double vr = speeds.omegaRadiansPerSecond;
        double v = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        Rotation2d direction = Rotation2d.fromRadians(Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond));

        Rotation2d filteredAngle = calculate(direction);

        double vx = Math.cos(filteredAngle.getRadians()) * v;
        double vy = Math.sin(filteredAngle.getRadians()) * v;

        return new ChassisSpeeds(vx, vy, vr);

    }

    /**
     * Set the rate limit of the filter
     * @param rateLimit new rate limit in radians per second
     */
    public void setRateLimit(double rateLimit){
        this.rateLimit = rateLimit;
    }

    /**
     * Filters the input to limit its slew rate.
     *
     * @param input The input value whose slew rate is to be limited.
     * @return The filtered value, which will not change faster than the slew rate.
     */
    public Rotation2d calculate(Rotation2d input) {
        double currentTime = MathSharedStore.getTimestamp(); //Time in seconds
        double elapsedTime = currentTime - prevTime;
        double deltaRadians = MathUtil.angleModulus(input.minus(prevVal).getRadians());

        //If the delta is too large, the robot is changing direction
        //Therefore the robot should first stop and reverse and then continue driving in the desired direction
        //To achieve this, the previous value is rotated by 180 degrees
        if (Math.abs(deltaRadians) > reverseThreshold.getRadians()){
            prevVal = prevVal.plus(Rotation2d.fromDegrees(180));
            //Calculate the delta again
            deltaRadians = MathUtil.angleModulus(input.minus(prevVal).getRadians());
        }

        double allowedIncreaseRadians = MathUtil.clamp(
                deltaRadians,
                -rateLimit * elapsedTime,
                rateLimit * elapsedTime
        );

        prevVal = prevVal.plus(Rotation2d.fromRadians(allowedIncreaseRadians));
        prevTime = currentTime;
        return prevVal;
    }
}
