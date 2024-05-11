package frc.team4481.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import frc.team4481.lib.subsystems.SubsystemManagerBase;

public class ClimberManager extends SubsystemManagerBase {

    private controlState currentControlState = controlState.DISABLED;

    public double climbingSpeed = 0;

    /**
     * Estimate of the robot pose by the Limelight stored in a {@code Pose2d}
     */
    private Pose2d[] limelightPoseEstimates;

    /**
     * Array containing the time at which the Limelight determined the pose
     */
    private double[] limelightReadingTimes;

    /**
     * Array storing the areas of the tags that the different Limelights see
     */
    private double[] tagAreas;

    public enum controlState {
        DISABLED,
        MANUAL,
        CALIBRATING,
    }


    public void setControlState(controlState pControlState) {
        currentControlState = pControlState;
    }
    public controlState getControlState() { return currentControlState; }

    public double getClimbingSpeed() {
        return climbingSpeed;
    }

    public void setClimbingSpeed(double climbingSpeed) {
        this.climbingSpeed = climbingSpeed;
    }


    public double[] getTagAreas() {
        return tagAreas;
    }

    public void setTagAreas(double[] tagAreas) {
        this.tagAreas = tagAreas;
    }

    public double[] getLimelightReadingTimes() {
        return limelightReadingTimes;
    }

    public void setLimelightReadingTimes(double[] limelightReadingTimes) {
        this.limelightReadingTimes = limelightReadingTimes;
    }

    public Pose2d[] getLimelightPoseEstimates() {
        return limelightPoseEstimates;
    }

    public void setLimelightPoseEstimates(Pose2d[] limelightPoseEstimates) {
        this.limelightPoseEstimates = limelightPoseEstimates;
    }


}
