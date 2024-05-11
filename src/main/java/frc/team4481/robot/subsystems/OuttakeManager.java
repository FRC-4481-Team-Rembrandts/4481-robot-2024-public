package frc.team4481.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.team4481.lib.subsystems.SubsystemManagerBase;

public class OuttakeManager extends SubsystemManagerBase {
    private controlState currentControlState = controlState.DISABLED;
    private movingState currentMovingState = movingState.MOVING;
    private positionState currentPositionState = positionState.STOWED;
    private boolean motorOverheat = false;

    private boolean isUpdated = false;

    private Pose2d robotPose = new Pose2d();
    private ChassisSpeeds robotSpeed = new ChassisSpeeds();

    public enum controlState {
        DISABLED,
        AUTOMATIC,
        MANUAL,

    }
    public enum movingState {
        ON_TARGET,
        MOVING,
    }

    public enum positionState{
        AGAINST_SUBWOOFER,
        PODIUM,
        AMP,
        INTAKING,
        STOWED,
        EJECT,
        OVER_STAGE,
        OVERRIDE_AUTOMATIC,
        AUTON_EJECT
    }

    public void setControlState(controlState pControlState) {
        currentControlState = pControlState;
        setUpdated(false);
    }
    public controlState getControlState() {
        return currentControlState;
    }
    public void setMovingState(movingState pMovingstate) {
        currentMovingState = pMovingstate;
    }
    public movingState getMovingState() {
        return currentMovingState;
    }
    public void setPositionState(positionState pPositionState) {
        currentPositionState = pPositionState;
        setUpdated(false);
    }
    public positionState getPositionState() {
        return currentPositionState;
    }
    public Pose2d getRobotPose(){
        return robotPose;
    }

    /**
     * with this position you can give a position to the shooter
     * @param pose is a the current position of the drivetrain
     */
    public void setRobotPose(Pose2d pose){
        this.robotPose = pose;
    }

    public boolean isUpdated() {
        return isUpdated;
    }

    public void setUpdated(boolean updated) {
        isUpdated = updated;
    }

    public void setMotorOverheat(boolean pOverheat) {
        motorOverheat = pOverheat;
    }
    public boolean getMotorOverheat() {return motorOverheat;}

    public ChassisSpeeds getRobotSpeed() {
        return robotSpeed;
    }

    public void setRobotSpeed(ChassisSpeeds robotSpeed) {
        this.robotSpeed = robotSpeed;
    }
}
