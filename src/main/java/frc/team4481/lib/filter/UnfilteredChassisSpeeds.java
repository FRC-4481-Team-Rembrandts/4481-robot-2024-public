package frc.team4481.lib.filter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class UnfilteredChassisSpeeds implements ChassisSpeedsFilter {

    private ChassisSpeeds speeds;

    @Override
    public ChassisSpeeds getFilteredSpeeds() {
        return speeds;
    }

    public void setBaseSpeeds(ChassisSpeeds speeds) {
        this.speeds = speeds;
    }
}
