package frc.team4481.robot.autoaim;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.lib.path.PoseToSpeedsConverter;
import frc.team4481.robot.Constants;
import frc.team4481.robot.util.ScoringHandler;

import static frc.team4481.robot.Constants.AutoAim.SUPERCYLE_TARGET_POSE_BLUE;
import static frc.team4481.robot.Constants.AutoAim.SUPERCYCLE_TARGET_POSE_RED;


public class TurnToPose implements PoseToSpeedsConverter {

    private final ScoringHandler scoringHandler = ScoringHandler.getInstance();
    private Translation2d deltaTranslation;
    private ChassisSpeeds rotationSpeed;
    private double pidMargin;
    private double setpointMargin;
    private double rotationKP;
    private boolean isInPidMargin;
    private boolean isInSetpointMargin;
    private ChassisSpeeds robotSpeed = new ChassisSpeeds();
    double velocityCompensationFactor;
    private PIDController pid;

    /**
     * Construct a new TurnToPose object to determine the rotational speed to look at a certain position
     * @param marginPid The margin in degrees within which the desired speed is 0
     * @param kP The constant with which the difference in angle is multiplied to determine the speed
     */
    public TurnToPose(double marginPid, double marginSetpoint, double kP, double kD, double velocityCompensationFactor) {
        pidMargin = marginPid;
        setpointMargin = marginSetpoint;

        rotationKP = kP;

        this.velocityCompensationFactor = velocityCompensationFactor;
        pid = new PIDController(kP,0, kD );
        pid.enableContinuousInput(-180, 180);
    }


    @Override
    public ChassisSpeeds getTargetSpeeds(Pose2d currentPose, PathPlannerTrajectory.State targetTrajectoryState) {
        return null;
    }

    @Override
    public ChassisSpeeds getTargetSpeeds(Pose2d currentPose, Pose2d targetPose) {
        //First, offset the current position based on the current velocity to compensate for movement during shot
        Pose2d correctedPose = compensateRobotPose(currentPose, robotSpeed);

        Rotation2d currentRotation = correctedPose.getRotation();

        // this hold the difference between the target and the current position of the robot in a translation (x,y)
        deltaTranslation = targetPose.getTranslation().minus(correctedPose.getTranslation());

        if (scoringHandler.getScoringPosition() == ScoringHandler.ScoringPosition.LOPJE){
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
                targetPose = SUPERCYLE_TARGET_POSE_BLUE;
            } else {
                targetPose = SUPERCYCLE_TARGET_POSE_RED;
            }
//            targetPose = new Pose2d(new Translation2d(targetPose.getX(), FIELD_WIDTH - 1), new Rotation2d()) ;
            deltaTranslation = targetPose.getTranslation().minus(correctedPose.getTranslation());
        }

        // determines the angle from the target to the '0 point'
        Rotation2d targetRotation = deltaTranslation.getAngle();

        // determines how much the robots needs to rotate
        Rotation2d rotateToTarget = targetRotation.minus(currentRotation);
        SmartDashboard.putNumber("error/error ratate", rotateToTarget.getDegrees());

        rotationSpeed = new ChassisSpeeds(0,0,0);
        isInSetpointMargin = (rotateToTarget.getDegrees() <= setpointMargin) && (rotateToTarget.getDegrees() >= -setpointMargin)
        && Math.abs(robotSpeed.omegaRadiansPerSecond) < Constants.AutoAim.AUTO_AIM_MAX_VEL_SETPOINT;
        isInPidMargin = (rotateToTarget.getDegrees() <= pidMargin) && (rotateToTarget.getDegrees() >= -pidMargin) ;
        if (!isInPidMargin) {
            // Pid control
            rotationSpeed.omegaRadiansPerSecond = pid.calculate(currentRotation.getDegrees(), targetRotation.getDegrees());
            // Feedforward control using simple trig
            rotationSpeed.omegaRadiansPerSecond += Math.atan((robotSpeed.vyMetersPerSecond - deltaTranslation.getY())/-deltaTranslation.getX()) - Math.atan(deltaTranslation.getY() / deltaTranslation.getX());
            rotationSpeed.omegaRadiansPerSecond = MathUtil.clamp(rotationSpeed.omegaRadiansPerSecond, -Constants.AutoAim.AUTO_AIM_MAX_ROT_VEL,Constants.AutoAim.AUTO_AIM_MAX_ROT_VEL);
        }
        SmartDashboard.putNumber("error/kp", rotationKP);
        SmartDashboard.putNumber("error/getDegree", rotateToTarget.getDegrees());
        SmartDashboard.putNumber("error/omegaRadians", rotationSpeed.omegaRadiansPerSecond);
        SmartDashboard.putNumber("error/vy", robotSpeed.vyMetersPerSecond);
        SmartDashboard.putBoolean("error/isinmargin", isInPidMargin);
        SmartDashboard.putNumber("error/getx", deltaTranslation.getX());
        SmartDashboard.putNumber("error/gety", deltaTranslation.getY());
        return rotationSpeed;
    }

    public boolean isInSetpointMargin() { return isInSetpointMargin; }

    public boolean isInPidMargin() {
        return isInPidMargin;
    }

    public void updateRobotSpeed(ChassisSpeeds robotSpeed){
        this.robotSpeed = robotSpeed;
    }

    private Pose2d compensateRobotPose(Pose2d robotPose, ChassisSpeeds robotSpeed){

        Translation2d deltaPos;

        deltaPos = new Translation2d(robotSpeed.vxMetersPerSecond*velocityCompensationFactor, robotSpeed.vyMetersPerSecond*velocityCompensationFactor);

        Translation2d newTrans = robotPose.getTranslation().plus(deltaPos);
        return new Pose2d(newTrans, robotPose.getRotation());
    }
}
