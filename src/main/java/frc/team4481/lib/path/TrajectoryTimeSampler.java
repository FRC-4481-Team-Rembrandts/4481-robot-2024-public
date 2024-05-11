package frc.team4481.lib.path;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;

public class TrajectoryTimeSampler implements TrajectoryToStateConverter{

    double startTime = 0;
    public TrajectoryTimeSampler(){
    }

    /**
     * Set the start time of the path
     * @param startTime start time in seconds
     */
    public void setStartTime(double startTime){
        this.startTime = startTime;
    }

    /** Function that samples a trajectory state from the path to drive to
     * @param trajectory Path planner trajectory that the robot is currently following
     * @param currentPose Current position of the robot
     * @param timestamp Time in seconds from the beginning of the path
     * @return {@code PathPlannerTrajectory.state} containing the information such as
     * the desired Pose2d and heading of the sampled point
     */
    @Override
    public PathPlannerTrajectory.State getTargetTrajectoryState(PathPlannerTrajectory trajectory, Pose2d currentPose, double timestamp) {
        return trajectory.sample(timestamp - startTime);
    }
}
