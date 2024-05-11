package frc.team4481.lib.path;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Can be used to aim a robot to a selected point. The translation will not be affected.
 */
public class DriveToTrajectoryState implements PoseToSpeedsConverter {

    private final PIDController xController;
    private final PIDController yController;
    private final ProfiledPIDController rotationController;
    private final double maxModuleSpeed;
    private final double mpsToRps;
    private final double accelFF;

    private StructPublisher<Pose2d> targetPosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Log poses/targetTrajectoryPose",Pose2d.struct).publish();


    public DriveToTrajectoryState(PIDConstants translationConstants, PIDConstants rotationConstants, double period, double maxModuleSpeed, double driveBaseRadius, double accelFF) {
        this.xController = new PIDController(translationConstants.kP, translationConstants.kI, translationConstants.kD, period);
        this.xController.setIntegratorRange(-translationConstants.iZone, translationConstants.iZone);
        this.yController = new PIDController(translationConstants.kP, translationConstants.kI, translationConstants.kD, period);
        this.yController.setIntegratorRange(-translationConstants.iZone, translationConstants.iZone);
        this.rotationController = new ProfiledPIDController(rotationConstants.kP, rotationConstants.kI, rotationConstants.kD, new TrapezoidProfile.Constraints(0.0, 0.0), period);
        this.rotationController.setIntegratorRange(-rotationConstants.iZone, rotationConstants.iZone);
        this.rotationController.enableContinuousInput(-3.141592653589793, Math.PI);
        this.maxModuleSpeed = maxModuleSpeed;
        this.mpsToRps = 1.0 / driveBaseRadius;
        this.accelFF = accelFF;
    }


    /**
     * @param currentPose           the current pose of the robot
     * @param targetState the desired position and speeds of the robot
     * @return the {@code ChassisSpeeds} containing the desired rotation speed to look to the target
     */
    @Override
    public ChassisSpeeds getTargetSpeeds(Pose2d currentPose, PathPlannerTrajectory.State targetState) {
        double xFF = targetState.velocityMps * targetState.heading.getCos();
        double yFF = targetState.velocityMps * targetState.heading.getSin();
        double xFFAccel = targetState.accelerationMpsSq * targetState.heading.getCos() * accelFF;
        double yFFAccel = targetState.accelerationMpsSq * targetState.heading.getSin() * accelFF;
        double xFeedback = this.xController.calculate(currentPose.getX(), targetState.positionMeters.getX());
        double yFeedback = this.yController.calculate(currentPose.getY(), targetState.positionMeters.getY());
        double angVelConstraint = targetState.constraints.getMaxAngularVelocityRps();
        double maxAngVelModule = Math.max(0.0, this.maxModuleSpeed - targetState.velocityMps) * this.mpsToRps;
        double maxAngVel = Math.min(angVelConstraint, maxAngVelModule);
        TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(maxAngVel, targetState.constraints.getMaxAngularAccelerationRpsSq());
        double targetRotationVel = this.rotationController.calculate(currentPose.getRotation().getRadians(), new TrapezoidProfile.State(targetState.targetHolonomicRotation.getRadians(), 0.0), rotationConstraints);

        // puts targetPose in smartdashboard
        Pose2d targetPose = new Pose2d(targetState.positionMeters.getX(), targetState.positionMeters.getY(), targetState.targetHolonomicRotation);
        targetPosePublisher.set(targetPose);

        Translation2d translation = new Translation2d(xFF, yFF);
        SmartDashboard.putNumber("DT/Desired Translation Speed (only feed forward)",translation.getNorm());
        SmartDashboard.putNumber("DT/Auto/Translation Error", targetPose.getTranslation().getDistance(currentPose.getTranslation()));

        return new ChassisSpeeds(xFFAccel + xFF + xFeedback, yFFAccel + yFF + yFeedback, targetRotationVel);
    }

    /**
     *  Converts {@code currentPose} and {@code targetPose} to {@code getTargetSpeeds}
     * @param currentPose the current pose of the robot
     * @param targetPose  the pose to aim towards
     * @return the {@code ChassisSpeeds} containing the desired rotation speed to look to the target
     */
    @Override
    public ChassisSpeeds getTargetSpeeds(Pose2d currentPose, Pose2d targetPose) {
        return null;
    }

    public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
        this.rotationController.reset(currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond);
    }
}
