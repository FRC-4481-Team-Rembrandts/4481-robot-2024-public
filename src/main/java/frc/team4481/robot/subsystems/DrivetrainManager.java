package frc.team4481.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.team4481.lib.subsystems.SubsystemManagerBase;

public class DrivetrainManager extends SubsystemManagerBase {

    private controlState currentControlState = controlState.DISABLED;
    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0,0,0);
    private ChassisSpeeds currentSpeeds = new ChassisSpeeds(0,0,0);
    private Pose2d poseEstimate = new Pose2d();
    private Pose2d poseToReset = new Pose2d();
    private boolean resetPoseUpdated = true;
    private boolean blinkLL = false;
    private boolean disableLL = false;

    public enum controlState {
        DISABLED,
        ENABLED,
        LOCKED,
    }

    public void setControlState(controlState pControlState) {
        currentControlState = pControlState;
    }
    public controlState getControlState() {
        return currentControlState;
    }

    public ChassisSpeeds getDesiredSpeeds() {
        return desiredSpeeds;
    }
    public void setDesiredSpeeds(ChassisSpeeds desiredSpeeds) {
        this.desiredSpeeds = desiredSpeeds;
    }

    public void setPoseEstimate(Pose2d poseEstimate){
        this.poseEstimate = poseEstimate;
    }
    public Pose2d getPoseEstimate() { return poseEstimate; }

    public ChassisSpeeds getCurrentSpeeds() {
        return currentSpeeds;
    }
    public void setCurrentSpeeds(ChassisSpeeds currentSpeeds) {
        this.currentSpeeds = currentSpeeds;
    }

    public Pose2d getPoseToReset() {
        return poseToReset;
    }
    public void setPoseToReset(Pose2d poseToReset) {
        this.poseToReset = poseToReset;
        setResetPoseUpdated(false);
    }

    public void setBlinkMode(boolean blinkLL) {
    this.blinkLL = blinkLL;
    }
    public  boolean getBlinkMode () {return  blinkLL;}

    public boolean isResetPoseUpdated() {
        return resetPoseUpdated;
    }
    public void setResetPoseUpdated(boolean resetPoseUpdated) {
        this.resetPoseUpdated = resetPoseUpdated;
    }

    public boolean isDisableLL() {
        return disableLL;
    }

    public void setDisableLL(boolean disableLL) {
        this.disableLL = disableLL;
    }
}