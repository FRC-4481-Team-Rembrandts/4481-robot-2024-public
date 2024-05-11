package frc.team4481.lib.path;

/**
 * Exception for when a {@code PathPlannerTrajectory} is requested but has not yet been preloaded.
 */
public class PathNotLoadedException extends Exception {

    /**
     * Throws new PathNotLoadedException for when a {@code PathPlannerTrajectory} is requested but does not exist
     * @param message error message to display
     */
    public PathNotLoadedException(String message) {
        super(message);
    }
}
