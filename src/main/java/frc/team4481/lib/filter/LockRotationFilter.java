package frc.team4481.lib.filter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LockRotationFilter extends FilterDecorator{

    private ChassisSpeeds filteredSpeed;
    private double angleMargin;
    private double adjustmentP;
    /**
     * Constructs a new FilterDecorator.
     *
     * @param chassisSpeedsFilter the filter to apply to the speeds
     */
    public LockRotationFilter(ChassisSpeedsFilter chassisSpeedsFilter, double angleMargin, double adjustmentP) {
        super(chassisSpeedsFilter);
        this.angleMargin = angleMargin;
        this.adjustmentP = adjustmentP;
    }

    public ChassisSpeeds getFilteredSpeeds() {
        //Return the filtered speeds that is updated by the update() function
        return filteredSpeed;
    }

    public void update(Rotation2d currentHeading, Rotation2d targetHeading, double dt){
        ChassisSpeeds unFilteredSpeed = tempFilter.getFilteredSpeeds();

        SmartDashboard.putNumber("Filters/locked robot heading", targetHeading.getDegrees());

        //Calculate the change in heading that is needed to achieve the target
        Rotation2d deltaHeading = targetHeading.minus(currentHeading);

        //Don't apply the filter if the error is within margin
        if (Math.abs(deltaHeading.getDegrees()) < angleMargin){
            filteredSpeed = unFilteredSpeed;
            return;
        }

        double correctedVr = deltaHeading.getRadians() / dt * adjustmentP;

        filteredSpeed = new ChassisSpeeds(unFilteredSpeed.vxMetersPerSecond, unFilteredSpeed.vyMetersPerSecond, correctedVr);


    }
}
