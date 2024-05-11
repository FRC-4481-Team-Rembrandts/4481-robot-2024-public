package frc.team4481.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.lib.hardware.Limelight;
import frc.team4481.lib.hardware.LimelightPoseEstimator;
import frc.team4481.lib.subsystems.SubsystemBase;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.lib.swerve.*;
import frc.team4481.robot.Constants;
import frc.team4481.robot.configuration.ConfigurationHandler;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.File;
import java.util.Optional;

import static frc.team4481.robot.Constants.AutoAim.*;
import static frc.team4481.robot.Constants.Field.FIELD_UPPER_RIGHT_CORNER;
import static frc.team4481.robot.Constants.HardwareMap.*;
import static frc.team4481.robot.Constants.Drivetrain.*;
import static frc.team4481.robot.Constants.*;
import static frc.team4481.robot.Constants.Vision.*;
import static frc.team4481.robot.Constants.Vision.TAG_AREA_THRESHOLD;


public class Drivetrain extends SubsystemBase<DrivetrainManager> {
    private final SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
    private final ConfigurationHandler configHandler = ConfigurationHandler.getInstance();

    public final Pigeon2 pigeon;

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final SwerveDriveKinematics kinematics;

    // Odometry
    private final SwerveDrivePoseEstimator poseEstimator;

    // PhotonVision
    private AprilTagFieldLayout aprilTagFieldLayout;
    private final PhotonCamera limelightFront;
    private final LimelightPoseEstimator cameraPoseEstimatorFront;
    private final PhotonCamera limelightBack;
    private final LimelightPoseEstimator cameraPoseEstimatorBack;
    private StructPublisher<Pose2d> robotPosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Vision/estimatedRobotPose", Pose2d.struct).publish();

    private final SwerveDrivetrainHelper swerveDrivetrainHelper;

    public final Field2d field = new Field2d();


    public Drivetrain() {
        name = "Drivetrain";
        subsystemManager = new DrivetrainManager();

        pigeon = new Pigeon2(PIGEON_IMU);

        //Setup the vision
//        aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        File LAYOUT_FILE = new File(Filesystem.getDeployDirectory(), "apriltaglayout/" + LAYOUT_FILE_NAME + ".json");

        try {
            aprilTagFieldLayout = new AprilTagFieldLayout(LAYOUT_FILE.toPath());
            DataLogManager.log("Using custom apriltag layout");
        } catch (Exception e) {
            e.printStackTrace();
            DataLogManager.log("Error: layout file is wrong -> fallback on default layout");
            aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        }

        limelightFront = new PhotonCamera("limelight-front");
        cameraPoseEstimatorFront = new LimelightPoseEstimator(aprilTagFieldLayout, limelightFront, CAMERA_FRONT_TRANSFORM, AMBIGUITY_THRESHOLD, TAG_AREA_THRESHOLD);
        limelightBack = new PhotonCamera("limelight-back");
        cameraPoseEstimatorBack = new LimelightPoseEstimator(aprilTagFieldLayout, limelightBack, CAMERA_BACK_TRANSFORM, AMBIGUITY_THRESHOLD, TAG_AREA_THRESHOLD);

        SwerveModule[] modules = new SwerveModule[4];

        for (int i = 0; i < modules.length; i++) {
            SwerveTurnHelper turnHelper = getSwerveTurnHelper(i);

            SwerveDriveHelper driveHelper = new SwerveDriveHelper(
                    DRIVE_IDS[i],
                    INVERTED[i],
                    DRIVE_POSITION_CONVERSION_FACTOR,
                    DRIVE_VELOCITY_CONVERSION_FACTOR,
                    DRIVE_PID_VALUES,
                    DRIVE_FF_VALUES,
                    STALL_LIMIT_DRIVE,
                    DRIVE_IDLE_MODE,
                    MODULE_TYPE == SWERVE_MODULE_TYPE.REV_VORTEX_NEO550
            );

            modules[i] = new SwerveModule(i, turnHelper, driveHelper);
        }

        frontLeft = modules[0];
        frontRight = modules[1];
        backLeft = modules[2];
        backRight = modules[3];

        kinematics = new SwerveDriveKinematics(
                FRONT_LEFT_LOCATION,
                FRONT_RIGHT_LOCATION,
                BACK_LEFT_LOCATION,
                BACK_RIGHT_LOCATION
        );

        swerveDrivetrainHelper = new SwerveDrivetrainHelper(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                kinematics
        );

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            poseEstimator = new SwerveDrivePoseEstimator(
                    kinematics,
                    Rotation2d.fromDegrees(pigeon.getYaw().getValue()),
                    swerveDrivetrainHelper.getSwerveModulePositions(),
                    DRIVETRAIN_START_POSITION_RED
            );
        } else {
            poseEstimator = new SwerveDrivePoseEstimator(
                    kinematics,
                    Rotation2d.fromDegrees(pigeon.getYaw().getValue()),
                    swerveDrivetrainHelper.getSwerveModulePositions(),
                    DRIVETRAIN_START_POSITION_BLUE
            );
        }

        swerveDrivetrainHelper.setMaxVelocity(configHandler.getConfig().getMaxDriveVelocity());
        swerveDrivetrainHelper.setFieldRelative(true);

        //Set standard deviation of vision measurements
        Matrix<N3, N1> stdDevsMatrix = new Matrix<>(Nat.N3(), Nat.N1());
        stdDevsMatrix.set(0,0,Vision.STANDARD_DEV_TRANSLATION);
        stdDevsMatrix.set(1,0,Vision.STANDARD_DEV_TRANSLATION);
        stdDevsMatrix.set(2,0,Vision.STANDARD_DEV_ROTATION);
        poseEstimator.setVisionMeasurementStdDevs(stdDevsMatrix);

        SmartDashboard.putData("Field", field);

        //Initialize the publishers for the robot pose to the smartdashboard
        robotPosePublisher = NetworkTableInstance.getDefault()
                .getStructTopic("Vision/estimatedRobotPose", Pose2d.struct).publish();

    }

    @Override
    public void onStart(double timestamp) {
                if (DriverStation.isAutonomous()) {
            subsystemManager.setControlState(DrivetrainManager.controlState.ENABLED);
        } else {
            subsystemManager.setControlState(DrivetrainManager.controlState.ENABLED);
        }

        zeroSensors();
    }

    @Override
    public void readPeriodicInputs() {
        if (!subsystemManager.isDisableLL()) {
            Optional<LimelightPoseEstimator.EstimatedPose2d> estimatedCameraFrontPose = cameraPoseEstimatorFront.getFilteredEstimatedPose(poseEstimator.getEstimatedPosition());
            if (estimatedCameraFrontPose.isPresent()) {
                // update pose estimator using front limelight
                poseEstimator.addVisionMeasurement(
                        estimatedCameraFrontPose.get().estimatedPose,
                        estimatedCameraFrontPose.get().timestampSeconds
                );
            }

            Optional<LimelightPoseEstimator.EstimatedPose2d> estimatedCameraBackPose = cameraPoseEstimatorBack.getFilteredEstimatedPose(poseEstimator.getEstimatedPosition());
            if (estimatedCameraBackPose.isPresent()) {
                // update pose estimator using back limelight
                poseEstimator.addVisionMeasurement(
                        estimatedCameraBackPose.get().estimatedPose,
                        estimatedCameraBackPose.get().timestampSeconds
                );
            }
        }

        //Update the pose estimator based on odometry
        poseEstimator.updateWithTime(
                Timer.getFPGATimestamp(),
                Rotation2d.fromDegrees(pigeon.getYaw().getValue()),
                swerveDrivetrainHelper.getSwerveModulePositions()
        );

        //Store the pose estimate in the manager
        subsystemManager.setPoseEstimate(poseEstimator.getEstimatedPosition());

        // Get distance to target speaker
        Translation2d targetPose;
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            targetPose = Constants.AutoAim.SPEAKER_TARGET_POSE_RED.getTranslation();
        } else {
            targetPose = Constants.AutoAim.SPEAKER_TARGET_POSE_BLUE.getTranslation();
        }
        SmartDashboard.putNumber("DT/bumper to subwoofer distance",
                subsystemManager.getPoseEstimate().getTranslation().getDistance(targetPose)
                        - DRIVETRAIN_WIDTH/2 - SUBWOOFER_WIDTH + SPEAKER_TARGET_X_OFFSET - SPEAKER_APRIL_TAG_RECESS);
    }

    @Override
    public void onLoop(double timestamp) {
        // limelight blinking
        if(subsystemManager.getBlinkMode()){
            limelightFront.setLED(VisionLEDMode.kBlink);
//            limelightBack.setLED(VisionLEDMode.kBlink);
        } else {
            limelightFront.setLED(VisionLEDMode.kOff);
//            limelightBack.setLED(VisionLEDMode.kOff);
        }

       switch (subsystemManager.getControlState()) {
            case DISABLED, LOCKED:
                swerveDrivetrainHelper.idleSwerveModuleStates();
                break;
            case ENABLED:
                //Check if the pose estimator should be reset
                if (!subsystemManager.isResetPoseUpdated()){
                    poseEstimator.resetPosition(
                            Rotation2d.fromDegrees(pigeon.getYaw().getValue()),
                            swerveDrivetrainHelper.getSwerveModulePositions(),
                            subsystemManager.getPoseToReset()
                    );
                    //Pass the reset pose to the manager
                    subsystemManager.setPoseEstimate(poseEstimator.getEstimatedPosition());
                    //Set updated to true
                    subsystemManager.setResetPoseUpdated(true);
                }

                //Update the current chassis speed
                subsystemManager.setCurrentSpeeds(getCurrentChassisSpeeds());

                //Update swerve modules based on desired chassis speeds and robot heading
                ChassisSpeeds desiredSpeeds = subsystemManager.getDesiredSpeeds();
                Rotation2d robotHeading = poseEstimator.getEstimatedPosition().getRotation();
                swerveDrivetrainHelper.updateSwerveModuleStates(desiredSpeeds, robotHeading);

                break;
        }
    }



    @Override
    public void writePeriodicOutputs() {

    }

    @Override
    public void onStop(double timestamp) {
        limelightFront.setLED(VisionLEDMode.kOff);
//        limelightBack.setLED(VisionLEDMode.kOff);
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void terminate() {
    }

    @Override
    public void outputData() {
        //Publish the estimated robot pose to the networktables
        robotPosePublisher.set(poseEstimator.getEstimatedPosition());

        SmartDashboard.putString("DT/Current ChassisSpeed", getCurrentChassisSpeeds().toString());
        SmartDashboard.putNumber("DT/Current Translation Speed", translationSpeed(getCurrentChassisSpeeds()));
        SmartDashboard.putNumber("DT/Desired Translation Speed", translationSpeed(subsystemManager.getDesiredSpeeds()));
        SmartDashboard.putNumber("DT/Current Pigeon yaw", pigeon.getYaw().getValue());
        field.setRobotPose(poseEstimator.getEstimatedPosition());

        SmartDashboard.putString("DT/controlState", subsystemManager.getControlState().toString());
        SmartDashboard.putString("DT/desiredspeeds", subsystemManager.getDesiredSpeeds().toString());

        PhotonPipelineResult cameraResult = limelightFront.getLatestResult();;
        for (int i = 0; i < cameraResult.targets.size(); i++) {
            PhotonTrackedTarget target = cameraResult.targets.get(i);
            SmartDashboard.putString("Vision/tag transform/ " + target.getFiducialId(), target.getBestCameraToTarget().toString());
        }
    }

    /**
     * Creates a swerve turn helper for each module
     *
     * @param i module number
     * @return SwerveTurnHelper
     */
    private SwerveTurnHelper getSwerveTurnHelper(int i) {
        SwerveTurnHelper turnHelper;

        // Is this drive train SDS?
        boolean isSDS = MODULE_TYPE == SWERVE_MODULE_TYPE.SDS_NEO_NEO;

        if (isSDS) {
            turnHelper = new SwerveTurnHelper(
                    TURN_IDS[i],
                    TURN_PID_VALUES,
                    TURN_FF_VALUES,
                    STALL_LIMIT_TURN,
                    TURN_ENCODER_IDS[i],
                    TURN_ENCODER_OFFSET_DEGREES[i],
                    TURN_GEAR_RATIO
            );
        } else {
            turnHelper = new SwerveTurnHelper(
                    TURN_IDS[i],
                    TURN_PID_VALUES,
                    TURN_FF_VALUES,
                    STALL_LIMIT_TURN
            );
        }
        return turnHelper;
    }

    /**
     * Retrieve the current FIELD RELATIVE chassis speed of the robot based on the encoder readings from the swerve modules
     * @return Current chassis speed of the robot
     */
    private ChassisSpeeds getCurrentChassisSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
                kinematics.toChassisSpeeds(swerveDrivetrainHelper.getSwerveModuleStates()),
                poseEstimator.getEstimatedPosition().getRotation()
        );
    }

    private double translationSpeed(ChassisSpeeds speeds) {
        Translation2d translation = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        return translation.getNorm();
    }

    private void SetIdleMode(CANSparkBase.IdleMode mode){
        backLeft.setIdleMode(mode);
        backRight.setIdleMode(mode);
        frontLeft.setIdleMode(mode);
        frontRight.setIdleMode(mode);
    }
}
