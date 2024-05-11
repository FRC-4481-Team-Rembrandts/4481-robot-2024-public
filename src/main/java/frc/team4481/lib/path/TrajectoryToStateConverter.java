package frc.team4481.lib.path;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * Interface for systems that sample an {@code PathplannerTrajectory.Sate} from an {@code PathPlannerTrajectory}
 */
public interface TrajectoryToStateConverter {
    PathPlannerTrajectory.State getTargetTrajectoryState(PathPlannerTrajectory trajectory, Pose2d currentPose, double timestamp);
}