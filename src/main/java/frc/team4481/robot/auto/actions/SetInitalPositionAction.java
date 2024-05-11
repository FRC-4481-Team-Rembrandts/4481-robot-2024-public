package frc.team4481.robot.auto.actions;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import frc.team4481.lib.auto.actions.Action;
import frc.team4481.lib.path.PathNotLoadedException;
import frc.team4481.lib.path.TrajectoryHandler;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.robot.subsystems.Drivetrain;
import frc.team4481.robot.subsystems.DrivetrainManager;

/**
 * Set the start position of the robot in autonomous, this action should be called at the beginning of EVERY
 * auto mode
 */
public class SetInitalPositionAction implements Action {
    private final String pathName;
    private String pathNameRed = null;
    private Pose2d initialPose;
    private final Drivetrain drivetrain;
    private final DrivetrainManager drivetrainManager;
    private boolean isFinished = false;

    /**
     * Reset the position of the drivetrain to the initial position of the path.
     * This action should be run before the first path that is followed.
     *
     * @param pathName Trajectory that the robot has to follow. Use the filename without extension from Path Planner
     */
    public SetInitalPositionAction(String pathName){
        this.pathName = pathName;

        SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
        drivetrain = (Drivetrain) subsystemHandler.getSubsystemByClass(Drivetrain.class);
        drivetrainManager = drivetrain.getSubsystemManager();
    }

    /**
     * Reset the position of the drivetrain to the initial position of the path.
     * This action should be run before the first path that is followed.
     *
     * @param pathNameBlue name of the path that the robot needs to follow on the BLUE alliance
     * @param pathNameRed name of the path that the robot needs to follow on the RED alliance
     */
    public SetInitalPositionAction(String pathNameBlue, String pathNameRed) {
        this(pathNameBlue);

        this.pathNameRed = pathNameRed;
    }

    /**
     * Reset the position of the drivetrain to (0,0)
     */
    public SetInitalPositionAction(){
        this(null);
    }

    @Override
    public void start() {
//        DataLogManager.log("Initial Position Action Start at: " + MathSharedStore.getTimestamp());

        //Enable the drivetrain
        drivetrainManager.setControlState(DrivetrainManager.controlState.ENABLED);

        if (pathName == null){
            drivetrainManager.setPoseToReset(new Pose2d());
            return;
        }


        //If path name is present, get the first position from the path
//        System.out.println("Time before path load: " + MathSharedStore.getTimestamp());
//        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
//        PathPlannerPath path = trajectoryHandler.setPath(pathName);

        TrajectoryHandler trajectoryHandler = TrajectoryHandler.getInstance();
        PathPlannerPath path = null;
        try {
            path = trajectoryHandler.getPath(pathName, pathNameRed);
        } catch (PathNotLoadedException e) {
            e.printStackTrace();
        }

        initialPose = path.getPreviewStartingHolonomicPose();

        //Tell the drivetrain manager that the drivetrain should reset its position
        drivetrainManager.setPoseToReset(initialPose);

    }

    @Override
    public void update() {
        //Force the drivetrain to be enabled
        drivetrainManager.setControlState(DrivetrainManager.controlState.ENABLED);

        //Check if the drivetrain has updated its position
        isFinished = drivetrainManager.isResetPoseUpdated();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void done() {
//        DataLogManager.log("Initial Position Action End at: "+ MathSharedStore.getTimestamp());
    }
}
