package frc.team4481.lib.path;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AdaptiveTrajectoryTimeSampler implements TrajectoryToStateConverter{

    double startTime = 0;
    double prevTime = 0;
    double timeOffset = 0;
    double maxError;


    /**
     * Construct a new adaptive time sampler of the trajectory.
     * This sampler will stop sampling further points if the error is too large
     * @param maxError The maximum error in meters before the sampler 'freezes' time
     */
    public AdaptiveTrajectoryTimeSampler(double maxError){
        this.maxError = maxError;
    }

    /**
     * Set the start time of the path
     * @param startTime start time in seconds
     */
    public void setStartTime(double startTime){
        this.startTime = startTime;
        prevTime = startTime;
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
        //Sample the trajectory based on time, keeping in account the time offset that has been set previously
        PathPlannerTrajectory.State sampledState = trajectory.sample(timestamp - startTime - timeOffset);
        //Get the coordinates of the sampled points as Translation2d
        Translation2d sampledTranslation = sampledState.positionMeters;
        //Calculate the distance of the robot to this sampled point
        double distanceToPoint = currentPose.getTranslation().getDistance(sampledTranslation);

        //Check if the distance to the target position is smaller than the maximum error
        if (distanceToPoint < maxError){
            //Everything is fine, update the previous time and return the sampled state
            prevTime = timestamp;
            return sampledState;
        }

        //Increase the offset to make sure the sampled point stays the same until the error is below the threshold again
        double dt = timestamp - prevTime;
        timeOffset += dt;

        //Update the previous time
        prevTime = timestamp;

        //Sample the trajectory again, but now with the increased offset
        PathPlannerTrajectory.State frozenState = trajectory.sample(timestamp - startTime - timeOffset);
        frozenState.velocityMps = 0;
        frozenState.accelerationMpsSq = 0;
        frozenState.headingAngularVelocityRps = 0;
        return frozenState;

    }

    /**
     * Return the timestamp that is currently used to sample the path.
     * This timestamp is relative to the start of the sampling and in seconds
     * @param timestamp Current time in seconds
     * @return Timestamp of the point in the path that is currently sampled
     */
    public double getCurrentSampledTime(double timestamp){
        return timestamp - startTime - timeOffset;
    }
}
