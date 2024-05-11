package frc.team4481.robot.auto.actions;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.lib.auto.actions.Action;
import frc.team4481.lib.path.*;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.robot.subsystems.Drivetrain;
import frc.team4481.robot.subsystems.DrivetrainManager;
import frc.team4481.robot.Constants;

/**
 * Action to drive a predefined path using the Pure Pursuit algorithm.
 */
public class DrivePathAction implements Action {
    private final String pathName;
    private String pathNameRed = null;

    private final Drivetrain drivetrain;
    private final DrivetrainManager drivetrainManager;
    private PathPlannerPath path;
    private PathPlannerTrajectory trajectory;

    private AdaptiveTrajectoryTimeSampler adaptiveTrajectoryTimeSampler;

    private DriveToTrajectoryState driveToTrajectoryState;
    private double startTime;
    private boolean isFinished = false;
    private double timeOut;
    private boolean flipPathForRed = true;

    /**
     * Create a new {@code DrivePathAction} that will follow a {@code PathPlannerPath}.
     *
     * @param pathName that the robot has to follow. Use the filename without extension from Path Planner
     */
    public DrivePathAction(String pathName){
        this.pathName = pathName;

        SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
        drivetrain = (Drivetrain) subsystemHandler.getSubsystemByClass(Drivetrain.class);
        drivetrainManager = drivetrain.getSubsystemManager();

        adaptiveTrajectoryTimeSampler = new AdaptiveTrajectoryTimeSampler(Constants.PathFollowing.MAX_PATH_ERROR);

        driveToTrajectoryState = new DriveToTrajectoryState(
                new PIDConstants(Constants.PathFollowing.DRIVE_CONTROLLER_XY_kP, 0, Constants.PathFollowing.DRIVE_CONTROLLER_XY_kD),
                new PIDConstants(Constants.PathFollowing.DRIVE_CONTROLLER_THETA_kP, 0, 0),
                Constants.kLooperDt,
                Constants.PathFollowing.MAX_VELOCITY,
                Constants.Drivetrain.DRIVETRAIN_WHEELBASE_DISTANCE*Math.sqrt(2),
                Constants.PathFollowing.ACCEL_FF
        );

        //Set the timeout to a very large number
        timeOut = 10000;
    }

    /**
     * Create a new {@code DrivePathAction} that will follow a {@code PathPlannerPath}
     * @param pathName name of the path that the robot needs to follow
     * @param timeOut delay in seconds after the action should be canceled
     */
    public DrivePathAction(String pathName, double timeOut){
        this(pathName);

        this.timeOut = timeOut;
    }

    /**
     * Create a new {@code DrivePathAction} that will follow a {@code PathPlannerPath}
     * @param pathNameBlue name of the path that the robot needs to follow on the BLUE alliance
     * @param timeOut delay in seconds after the action should be canceled
     * @param pathNameRed name of the path that the robot needs to follow on the RED alliance
     */
    public DrivePathAction(String pathNameBlue, double timeOut, String pathNameRed) {
        this(pathNameBlue, timeOut);

        this.pathNameRed = pathNameRed;
    }

    @Override
    public void start() {
        SmartDashboard.putString("Auto/trajectory name", pathName);

        // Get Current drivetrain info
        ChassisSpeeds startingSpeeds = drivetrainManager.getCurrentSpeeds();
        Pose2d currentPose = drivetrainManager.getPoseEstimate();

        startTime = MathSharedStore.getTimestamp();

        adaptiveTrajectoryTimeSampler.setStartTime(startTime);

        //Get the path object based on the string of the path name
        TrajectoryHandler trajectoryHandler = TrajectoryHandler.getInstance();
        PathPlannerPath path = null;
        try {
            path = trajectoryHandler.getPath(pathName, pathNameRed);
        } catch (PathNotLoadedException e) {
            e.printStackTrace();
        }

        //Convert this path to a trajectory that the robot can actually follow
        trajectory = new PathPlannerTrajectory(path, startingSpeeds, currentPose.getRotation());
        //Reset the drive to trajectory state object
        driveToTrajectoryState.reset(drivetrainManager.getPoseEstimate(), drivetrainManager.getCurrentSpeeds());

        //Set the drivetrain to enabled
        drivetrainManager.setControlState(DrivetrainManager.controlState.ENABLED);
    }

    @Override
    public void update() {
        Pose2d currentPose = drivetrainManager.getPoseEstimate();
        double timestamp = MathSharedStore.getTimestamp();
        //Get the target state that the robot needs to drive to
        PathPlannerTrajectory.State targetState = adaptiveTrajectoryTimeSampler.getTargetTrajectoryState(trajectory, currentPose, timestamp);
        //Convert this state into useable chassis speeds
        ChassisSpeeds targetChassisSpeeds = driveToTrajectoryState.getTargetSpeeds(currentPose, targetState);

        drivetrainManager.setDesiredSpeeds(targetChassisSpeeds);

        //Check if finished, this happens if the time of the point that is sampled is larger than the duration of the path
        //Because the adaptive sampler adds an offset, the current time cannot directly be used
        //Also check if the time that has passed is larger than the time out delay
        if (adaptiveTrajectoryTimeSampler.getCurrentSampledTime(timestamp) > trajectory.getTotalTimeSeconds()
            || timestamp - startTime > timeOut) {
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void done() {
        drivetrainManager.setControlState(DrivetrainManager.controlState.DISABLED);
    }
}
