package frc.team4481.robot.autoaim;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.lib.path.PoseToSpeedsConverter;
import frc.team4481.robot.Constants;
import frc.team4481.robot.Constants.*;


/**
 * Can be used to aim a robot to a selected point. The translation will not be affected.
 */
public class DriveToPose implements PoseToSpeedsConverter {
    Transform2d deltaPose;
    double targetPoseX;
    double currentPoseX;
    double deltaX;
    double speedX;
    double targetPoseY;
    double currentPoseY;
    double deltaY;
    double speedY;
    double targetPoseRotation;
    double currentPoseRotation;
    double deltaRotation;
    double speedRotation;
    double kPtranslation;
    double kProtation;
    double translationMarge;
    double rotationMarge;
    private ProfiledPIDController pidDriveToPosX;
    private ProfiledPIDController pidDriveToPosY;
    private TrapezoidProfile.Constraints pidConstants;
    private StructPublisher<Pose2d> autoAmpPosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Vision/autoAmpRobotPose", Pose2d.struct).publish();
    private double lastResetTime = -10;
    double pidPosCorrectieX, pidPosCorrectieY;
    private ChassisSpeeds robotSpeed = new ChassisSpeeds();

    private final double TARGET_IN_RANGE_THRESHOLD = 1.0; // m
    private final double TARGET_OUT_OF_RANGE_Y_OFFSET = 0.5;// m
    private final double PID_PROFILE_MAX_VELOCITY = 4.5; // m/s
    private final double PID_PROFILE_MAX_ACCELERATION = 4; // m/s^2


    public DriveToPose(double kPtranslation, double kProtation, double translationMarge, double rotationMarge){
        this.kPtranslation = kPtranslation;
        this.kProtation = kProtation;
        this.translationMarge = translationMarge;
        this.rotationMarge = rotationMarge;

        autoAmpPosePublisher = NetworkTableInstance.getDefault()
                .getStructTopic("Vision/autoAmpRobotPose", Pose2d.struct).publish();

        pidConstants = new TrapezoidProfile.Constraints(PID_PROFILE_MAX_VELOCITY, PID_PROFILE_MAX_ACCELERATION);
        pidDriveToPosX = new ProfiledPIDController(
                kPtranslation,0,0,
                pidConstants, Constants.kHIDLooperDt);
        pidDriveToPosY = new ProfiledPIDController(
                kPtranslation, 0,0,
                pidConstants, Constants.kHIDLooperDt);

    }

    /**
     * @param currentPose           the current pose of the robot
     * @param targetTrajectoryState the desired position and speeds of the robot
     * @return the {@code ChassisSpeeds} containing the desired rotation speed to look to the target
     */
    @Override
    public ChassisSpeeds getTargetSpeeds(Pose2d currentPose, PathPlannerTrajectory.State targetTrajectoryState) {
        return null;
    }

    /**
     *  converts {@code currentPose} and {@code targetPose} to {@code getTargetSpeeds}
     * @param currentPose the current pose of the robot
     * @param targetPose  the pose to aim towards
     * @return the {@code ChassisSpeeds} containing the desired rotation speed to look to the target
     */
    @Override
    public ChassisSpeeds getTargetSpeeds(Pose2d currentPose, Pose2d targetPose){ //ChassisSpeeds currentSpeeds) {
        //reset of the pid
        if (1 <= (MathSharedStore.getTimestamp() - lastResetTime)) {
            pidDriveToPosX.reset(currentPose.getX(), robotSpeed.vxMetersPerSecond);
            pidDriveToPosY.reset(currentPose.getY(), robotSpeed.vyMetersPerSecond);
            lastResetTime = MathSharedStore.getTimestamp();

        } else {
            lastResetTime = MathSharedStore.getTimestamp();
        }

        targetPoseX = targetPose.getX();
        currentPoseX = currentPose.getX();
        deltaX = targetPoseX - currentPoseX;

        targetPoseY = targetPose.getY();
        currentPoseY = currentPose.getY();
        deltaY = targetPoseY - currentPoseY;

        // If the X position of the robot is not in range of the target
        // then we offset the target Y position
        if(Math.abs(deltaX) > TARGET_IN_RANGE_THRESHOLD){
            targetPoseY -= TARGET_OUT_OF_RANGE_Y_OFFSET;
        }

        pidDriveToPosX.setGoal(targetPoseX);
        pidPosCorrectieX = pidDriveToPosX.calculate(currentPoseX, targetPoseX);
        double profileVX = pidDriveToPosX.getSetpoint().velocity;
        speedX = pidPosCorrectieX;

        pidDriveToPosY.setGoal(targetPoseY);
        pidPosCorrectieY = pidDriveToPosY.calculate(currentPoseY, targetPoseY);
        double profileVY = pidDriveToPosY.getSetpoint().velocity;
        speedY = pidPosCorrectieY;

        SmartDashboard.putNumber("DT/auto_amping/pidX", pidPosCorrectieX);
        SmartDashboard.putNumber("DT/auto_amping/pidY", pidPosCorrectieY);
        SmartDashboard.putNumber("DT/auto_amping/profileVX", profileVX);
        SmartDashboard.putNumber("DT/auto_amping/profileVY", profileVY);
        SmartDashboard.putNumber("DT/auto_amping/speedx", speedX);
        SmartDashboard.putNumber("DT/auto_amping/speedy", speedY);
        SmartDashboard.putNumber("DT/auto_amping/frfrVX", robotSpeed.vxMetersPerSecond);
        SmartDashboard.putNumber("DT/auto_amping/frfrVY", robotSpeed.vyMetersPerSecond);

        targetPoseRotation = targetPose.getRotation().getRadians();
        currentPoseRotation = currentPose.getRotation().getRadians();
        //Set deltaRotation in Radians
        deltaRotation = targetPoseRotation - currentPoseRotation;
        //Calculate the smallest angle, so for example 340 becomes 20
        deltaRotation = MathUtil.angleModulus(deltaRotation);
        //Change back to degrees
        deltaRotation = Math.toDegrees(deltaRotation);
        if (Math.abs(deltaRotation) <= rotationMarge) {
            speedRotation = 0;
        } else {
            speedRotation = deltaRotation * kProtation;
        }

        autoAmpPosePublisher.set(targetPose);

        //Convert to chassis speed and return
        return new ChassisSpeeds(speedX, speedY, speedRotation);
    }

    public void setRobotSpeed(ChassisSpeeds robotSpeed) {
        this.robotSpeed = robotSpeed;
    }
}
