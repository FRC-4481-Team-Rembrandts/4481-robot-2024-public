package frc.team4481.lib.path;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Interface for systems that can convert a difference in {@code Pose2d} objects to a {@code ChassisSpeeds}
 */
public interface PoseToSpeedsConverter {
    /**
     * Converts a target {@code targetTrajectoryState} and current {@code Pose2d} into an appropriate {@code ChassisSpeeds}
     *
     * @param currentPose the current pose of the robot
     * @param targetTrajectoryState the desired position and speeds of the robot
     * @return the {@code ChassisSpeeds} to get from the current pose to the target pose
     */
    ChassisSpeeds getTargetSpeeds(Pose2d currentPose, PathPlannerTrajectory.State targetTrajectoryState);

    /**
     * Converts a target and current {@code Pose2d} into an appropriate {@code ChassisSpeeds}
     *
     * @param currentPose the current pose of the robot
     * @param targetPose the desired position of the robot
     * @return the {@code ChassisSpeeds} to get from the current pose to the target pose
     */
    ChassisSpeeds getTargetSpeeds(Pose2d currentPose, Pose2d targetPose);
}
