package frc.team4481.lib.path;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.HashMap;
import java.util.List;


/**
 * Utility class to load and transform a {@code PathPlannerPath}.
 */
public class TrajectoryHandler {
    private static TrajectoryHandler instance;

    private PathPlannerPath currentPath;
    private final HashMap<String, PathPlannerPath> loadedPathMap = new HashMap<>();
    private final HashMap<String, PathPlannerTrajectory> loadedPPTrajectoryMap = new HashMap<>();

    private TrajectoryHandler() {}

    /**
     * Gets the {@code TrajectoryHandler} instance.
     *
     * @return singleton instance of the {@code TrajectoryHandler}
     */
    public static TrajectoryHandler getInstance() {
        if (instance == null)
            instance = new TrajectoryHandler();

        return instance;
    }

    /**
     * Preloads a Path Planner 2 path for faster processing.
     * This method is intended to be called in the disabled state of the robot.
     *
     * @param pathName      Filename of the path minus file extension
     */
    private void preloadPath(
            String pathName,
            boolean flipped
    ) {
        PathPlannerPath path;
        if (flipped) {
            path = PathPlannerPath.fromPathFile(pathName).flipPath();
            pathName = pathName + "flipped";
        } else {
            path = PathPlannerPath.fromPathFile(pathName);
        }
        if (!loadedPathMap.containsKey(pathName)) {
            DataLogManager.log(pathName + " is preloaded");
            loadedPathMap.put(
                    pathName,
                    path
            );

            // TODO @Jochem please test
//            Rotation2d startingRotation = path.getAllPathPoints().getFirst().rotationTarget.getTarget();
//            ChassisSpeeds startingSpeeds = new ChassisSpeeds(path.getAllPathPoints().getFirst().maxV, 0, 0);
//            PathPlannerTrajectory pathPlannerTrajectory = path.getTrajectory(startingSpeeds, startingRotation);

            PathPlannerTrajectory pathPlannerTrajectory = path.getTrajectory(new ChassisSpeeds(), new Rotation2d());
            loadedPPTrajectoryMap.put(
                    pathName,
                    pathPlannerTrajectory
            );
            // Convert PathPlannerStates to TrajectoryStates
            List<Trajectory.State> trajectoryStates = pathPlannerTrajectory.getStates().stream().map(TrajectoryHandler::convertPathPlannerStateToTrajectoryState).toList();
            // Log trajectories
            NetworkTableInstance.getDefault().getProtobufTopic("AutoPaths/" + pathName, Trajectory.proto).publish().set(new Trajectory(trajectoryStates));
        }
    }

    /**
     * Gets the current {@code PathPlannerPath}.
     *
     * @return current {@code PathPlannerPath}
     */
    public PathPlannerPath getCurrentPath() {
        return currentPath;
    }

    /**
     * Transforms a Path Planner 2 path into a {@code PathPlannerPath}. Preloading a path increases loading speed
     *
     * @param pathName      Filename of the path minus file extension
     *
     * @see TrajectoryHandler#preloadPath(String) preloading trajectories
     */
//    public PathPlannerPath setPath(
//            String pathName
//    ) {
//        try {
//            if (loadedPathMap.containsKey(pathName))
//                return setPreloadedPath(pathName, false);
//            return setUnloadedPath(pathName);
//        } catch (PathNotLoadedException e) {
//            e.printStackTrace();
//        }
//
//        return null;
//    }

    /**
     * Sets a preloaded {@code PathPlannerPath} as current trajectory.
     *
     * @param pathName name of the path as noted in Path Planner 2
     * @throws PathNotLoadedException if the path has not been loaded yet
     * @return the currently loaded trajectory
     */
    public PathPlannerPath getPreloadedPath(String pathName, boolean flipped) throws PathNotLoadedException {
        if (flipped) {
            pathName = pathName + "flipped";
        }

        if (!loadedPathMap.containsKey(pathName))
            throw new PathNotLoadedException("Path '" + pathName + "' has not been preloaded yet.");

        currentPath = loadedPathMap.get(pathName);

        return currentPath;
    }

    public PathPlannerTrajectory getPreloadedTrajectory(String pathName, boolean flipped) throws PathNotLoadedException {
        if (flipped) {
            pathName = pathName + "flipped";
        }

        if (!loadedPPTrajectoryMap.containsKey(pathName))
            throw new PathNotLoadedException("Path '" + pathName + "' has not been preloaded yet.");

        return loadedPPTrajectoryMap.get(pathName);
    }

    public PathPlannerPath getPath(String pathName, String pathNameRed) throws PathNotLoadedException {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            if (pathNameRed == null) {
                return getPreloadedPath(pathName, true);
            } else {
                return getPreloadedPath(pathNameRed, false);
            }
        } else {
            return getPreloadedPath(pathName, false);
        }

    }

    public PathPlannerTrajectory getTrajectory(String pathName, String pathNameRed) throws PathNotLoadedException {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            if (pathNameRed == null) {
                return getPreloadedTrajectory(pathName, true);
            } else {
                return getPreloadedTrajectory(pathNameRed, false);
            }
        } else {
            return getPreloadedTrajectory(pathName, false);
        }

    }

    /**
     * Sets an unloaded {@code PathPlannerTrajectory} as current trajectory and loads it for future use.
     *
     * @param pathName name of the path as noted in Path Planner 2
     * @throws PathNotLoadedException if the path has not been loaded yet
     * @return the currently loaded trajectory
     */
    public void setUnloadedPath(
            String pathName
    ) throws PathNotLoadedException {
        preloadPath(pathName, false);
        preloadPath(pathName, true);
    }

    public void setUnloadedPath(
            String pathNameBlue,
            String pathNameRed
    ) throws PathNotLoadedException {
        preloadPath(pathNameBlue, false);
        preloadPath(pathNameRed, false);
    }

    /**
     * Convert {@code PathPlannerTrajectory.State} to {@code Trajectory.State}
     *
     * @param pathPlannerState
     * @return trajectoryState
     */
    public static Trajectory.State convertPathPlannerStateToTrajectoryState(PathPlannerTrajectory.State pathPlannerState) {
        return new Trajectory.State(
                pathPlannerState.timeSeconds,
                pathPlannerState.velocityMps,
                pathPlannerState.accelerationMpsSq,
                new Pose2d(pathPlannerState.positionMeters, pathPlannerState.targetHolonomicRotation),
                pathPlannerState.curvatureRadPerMeter
            );
    }

}
