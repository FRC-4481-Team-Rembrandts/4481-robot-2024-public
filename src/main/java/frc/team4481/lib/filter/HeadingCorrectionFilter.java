package frc.team4481.lib.filter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.lib.util.CountingDelay;

/**
 * A filter that corrects the heading of the robot.
 */
public class HeadingCorrectionFilter extends FilterDecorator {

    private Rotation2d targetHeading;
    private ChassisSpeeds filteredSpeed;
    private CountingDelay turnOndelay;
    private double angleMargin;
    private double adjustmentP;
    private double turnOnDelayTime;

    /**
     * Constructs a new HeadingCorrectionFilter.
     * @param filter the filter to apply to the speeds
     * @param angleMargin The margin that the correction filter has
     * @param adjustmentP The P value for the adjustment
     * @param turnOnDelayTime The time in seconds it takes for the filter to turn on
     */
    public HeadingCorrectionFilter(ChassisSpeedsFilter filter, double angleMargin, double adjustmentP, double turnOnDelayTime) {
        super(filter);
        this.angleMargin = angleMargin;
        this.adjustmentP = adjustmentP;
        this.turnOnDelayTime = turnOnDelayTime;

        turnOndelay = new CountingDelay();

        targetHeading = new Rotation2d();

    }

    /**
     * Returns the filtered speeds.
     * @return the filtered speeds
     */
    public ChassisSpeeds getFilteredSpeeds() {
        ChassisSpeeds speeds = tempFilter.getFilteredSpeeds();

        //Return the filtered speeds that is updated by the update() function
        return filteredSpeed;
    }


    /**
     * Update the filter to determine the heading corrected chassis speeds.
     * This should always be called, otherwise the filter does not work.
     * @param currentHeading Which angle the front of the robot is currently facing
     * @param dt Cycle time of the code
     */
    public void update(Rotation2d currentHeading, double dt){
        //Retrieve the unfiltered speeds, these might already have been filtered by other filters
        ChassisSpeeds unFilteredSpeed = tempFilter.getFilteredSpeeds();

        //Get desired rotational speed in radians per second and absolute translational speed in m/s
        double vr = unFilteredSpeed.omegaRadiansPerSecond;
        double v = Math.hypot(unFilteredSpeed.vxMetersPerSecond, unFilteredSpeed.vyMetersPerSecond);

        SmartDashboard.putNumber("Filters/target robot heading", targetHeading.getDegrees());

        //Check if the correction should be applied
        //Don't apply correction if there is already a desired rotation speed
        if (vr > 0.01 || vr < -0.01){
            targetHeading = currentHeading;
            filteredSpeed = unFilteredSpeed;
            return;
        }
        //If there is no more desired rotation, wait some time to turn on again
        if (!turnOndelay.delay(turnOnDelayTime)){
            targetHeading = currentHeading;
            filteredSpeed = unFilteredSpeed;
            return;
        }
        //If the delay has passed, reset the delay object to make sure it stays true
        turnOndelay.reset();

        //Don't apply correction if the desired translation speed is near zero
        if (v < 0.1){
            targetHeading = currentHeading;
            filteredSpeed = unFilteredSpeed;
            return;
        }

        //Determine what the desired heading of the robot is by extrapolating the current desired heading
        targetHeading = targetHeading.plus(new Rotation2d(vr * dt));

        //Calculate the change in heading that is needed to achieve the target
        Rotation2d deltaHeading = targetHeading.minus(currentHeading);

        //Don't apply the filter if the error is within margin
        if (Math.abs(deltaHeading.getDegrees()) < angleMargin){
            filteredSpeed =  unFilteredSpeed;
            return;
        }

        double correctedVr = deltaHeading.getRadians() / dt * adjustmentP;

        filteredSpeed = new ChassisSpeeds(unFilteredSpeed.vxMetersPerSecond, unFilteredSpeed.vyMetersPerSecond, correctedVr);

    }
}
