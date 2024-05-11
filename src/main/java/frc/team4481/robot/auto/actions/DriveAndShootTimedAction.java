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
import frc.team4481.lib.path.AdaptiveTrajectoryTimeSampler;
import frc.team4481.lib.path.DriveToTrajectoryState;
import frc.team4481.lib.path.PathNotLoadedException;
import frc.team4481.lib.path.TrajectoryHandler;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.robot.Constants;
import frc.team4481.robot.autoaim.TurnToPose;
import frc.team4481.robot.subsystems.*;

/**
 * Action to drive a predefined path using the Pure Pursuit algorithm.
 */
public class DriveAndShootTimedAction implements Action {
    private final String pathName;
    private String pathNameRed = null;

    private final Drivetrain drivetrain;
    private final DrivetrainManager drivetrainManager;
    private final Intake intake;
    private final IntakeManager intakeManager;
    private final Outtake outtake;
    private final OuttakeManager outtakeManager;

//    private PathPlannerPath path;
    private PathPlannerTrajectory trajectory;
    private AdaptiveTrajectoryTimeSampler adaptiveTrajectoryTimeSampler;
    private DriveToTrajectoryState driveToTrajectoryState;
    private double startTime;
    private boolean isFinished = false;
    private double timeOut;
    private double[] shootTime;
    private boolean flipPathForRed = true;


    private TurnToPose turnToPose;
    private Pose2d targetPose;

    private final double TURBO_FEEDING_TIME = 0.5;
    /**
     * Create a new {@code DrivePathAction} that will follow a {@code PathPlannerPath}.
     *
     * @param pathName that the robot has to follow. Use the filename without extension from Path Planner
     */
    public DriveAndShootTimedAction(String pathName){
        this.pathName = pathName;

        SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
        drivetrain = (Drivetrain) subsystemHandler.getSubsystemByClass(Drivetrain.class);
        drivetrainManager = drivetrain.getSubsystemManager();
        intake = (Intake) subsystemHandler.getSubsystemByClass(Intake.class);
        intakeManager = intake.getSubsystemManager();
        outtake = (Outtake) subsystemHandler.getSubsystemByClass(Outtake.class);
        outtakeManager = outtake.getSubsystemManager();

        adaptiveTrajectoryTimeSampler = new AdaptiveTrajectoryTimeSampler(Constants.PathFollowing.MAX_PATH_ERROR);

        driveToTrajectoryState = new DriveToTrajectoryState(
                new PIDConstants(Constants.PathFollowing.DRIVE_CONTROLLER_XY_kP, 0, Constants.PathFollowing.DRIVE_CONTROLLER_XY_kD),
                new PIDConstants(Constants.PathFollowing.DRIVE_CONTROLLER_THETA_kP, 0, 0),
                Constants.kLooperDt,
                Constants.PathFollowing.MAX_VELOCITY,
                Constants.Drivetrain.DRIVETRAIN_WHEELBASE_DISTANCE*Math.sqrt(2),
                Constants.PathFollowing.ACCEL_FF
        );

        turnToPose = new TurnToPose(Constants.AutoAim.AUTO_AIM_MARGIN,
                Constants.AutoAim.AUTO_AIM_SETPOINT_MARGIN,
                Constants.AutoAim.AUTO_AIM_KP,
                Constants.AutoAim.AUTO_AIM_KD,
                Constants.AutoAim.ROTATION_VELOCITY_COMPENSATION_FACTOR);

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            targetPose = Constants.AutoAim.SPEAKER_TARGET_POSE_RED;
        } else {
            targetPose = Constants.AutoAim.SPEAKER_TARGET_POSE_BLUE;
        }


        //Set the timeout to a very large number
        timeOut = 10000;
        shootTime = new double[]{10000};
    }

    /**
     * Create a new {@code DrivePathAction} that will follow a {@code PathPlannerPath}
     * @param pathName name of the path that the robot needs to follow
     * @param shootTime when to make the shot
     */
    public DriveAndShootTimedAction(String pathName, double[] shootTime){
        this(pathName);

        this.shootTime = shootTime;
    }

    public DriveAndShootTimedAction(String pathName, double[] shootTime, double timeOut){
        this(pathName);

        this.timeOut = timeOut;
        this.shootTime = shootTime;
    }

    /**
     * Create a new {@code DrivePathAction} that will follow a {@code PathPlannerPath}
     * @param pathNameBlue name of the path that the robot needs to follow on the BLUE alliance
     * @param shootTime delay in seconds after the action should be canceled
     * @param pathNameRed name of the path that the robot needs to follow on the RED alliance
     */
    public DriveAndShootTimedAction(String pathNameBlue, double[] shootTime, String pathNameRed) {
        this(pathNameBlue, shootTime);

        this.pathNameRed = pathNameRed;
    }

    @Override
    public void start() {
        DataLogManager.log("Drive Shoot Action Start at: " + MathSharedStore.getTimestamp());
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

        //Set the control state to moving
        outtakeManager.setControlState(OuttakeManager.controlState.AUTOMATIC);
        if (intakeManager.getControlState() != IntakeManager.controlState.HOLD && intakeManager.getControlState() != IntakeManager.controlState.STORE) {
            intakeManager.setControlState(IntakeManager.controlState.INTAKE);

        }
    }

    @Override
    public void update() {

        drivetrainManager.setControlState(DrivetrainManager.controlState.ENABLED);
        Pose2d currentPose = drivetrainManager.getPoseEstimate();
        double timestamp = MathSharedStore.getTimestamp();
        //Get the target state that the robot needs to drive to
        PathPlannerTrajectory.State targetState = adaptiveTrajectoryTimeSampler.getTargetTrajectoryState(trajectory, currentPose, timestamp);

        //Convert this state into useable chassis speeds
        ChassisSpeeds pathChassisSpeeds = driveToTrajectoryState.getTargetSpeeds(currentPose, targetState);
        turnToPose.updateRobotSpeed(drivetrainManager.getCurrentSpeeds());
        ChassisSpeeds aimingChassisSpeeds = turnToPose.getTargetSpeeds(currentPose, targetPose);
        ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds();
        targetChassisSpeeds.vyMetersPerSecond = pathChassisSpeeds.vyMetersPerSecond;
        targetChassisSpeeds.vxMetersPerSecond = pathChassisSpeeds.vxMetersPerSecond;

        //Pass the robot pose from the drivetrain to the outtake
        outtakeManager.setRobotPose(drivetrainManager.getPoseEstimate());
        outtakeManager.setRobotSpeed(drivetrainManager.getCurrentSpeeds());

        boolean autoAiming;
//        if (intakeManager.getControlState() == IntakeManager.controlState.TURBO_FEEDER ||
//        intakeManager.getControlState() == IntakeManager.controlState.HOLD){
//            //If the intake is holding a note or feeding, it should aim to the speaker
//            targetChassisSpeeds.omegaRadiansPerSecond = aimingChassisSpeeds.omegaRadiansPerSecond;
//            autoAiming = true;
//
//        } else {
//            targetChassisSpeeds.omegaRadiansPerSecond = pathChassisSpeeds.omegaRadiansPerSecond;
//            autoAiming = false;
//        }
        targetChassisSpeeds.omegaRadiansPerSecond = pathChassisSpeeds.omegaRadiansPerSecond;

        //SmartDashboard.putBoolean("DT/Auto/auto aiming", autoAiming);



        drivetrainManager.setDesiredSpeeds(targetChassisSpeeds);
        SmartDashboard.putNumber("DT/TargetSpeeds", targetChassisSpeeds.omegaRadiansPerSecond);
        double sampletime = adaptiveTrajectoryTimeSampler.getCurrentSampledTime(timestamp);

        for (double time : shootTime) {
            if (sampletime > time && sampletime <(time + TURBO_FEEDING_TIME)) {
                intakeManager.setControlState(IntakeManager.controlState.TURBO_FEEDER);
            }
            if (sampletime > (time + TURBO_FEEDING_TIME) && intakeManager.getControlState()==IntakeManager.controlState.TURBO_FEEDER){
                intakeManager.setControlState(IntakeManager.controlState.INTAKE);
            }
        }



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
        DataLogManager.log("Drive Action Done");
    }
}
