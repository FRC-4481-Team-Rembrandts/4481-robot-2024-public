package frc.team4481.robot.subsystems.modules;

import frc.team4481.lib.subsystems.SubsystemManagerBase;
import frc.team4481.robot.Constants;

public class PivotManager extends SubsystemManagerBase {

    private controlState currentControlState = controlState.DISABLED;
    private movingState currentMovingState = movingState.MOVING;
    private positionState currentPositionState = positionState.STOWED;
    private double setPointMargin = 1000;

    private double automaticAngleTarget = 0.0;
    private double absoluteAngle = 0.0;

    public enum controlState {
        DISABLED,
        AUTOMATIC,
        MANUAL,

    }

    public enum movingState {
        MOVING,
        ON_TARGET,
    }
    public enum positionState{
        AGAINST_SPEAKER(Constants.Pivot.PIVOT_ANGLE_AGAINST_SPEAKER),
        PODIUM(Constants.Pivot.PIVOT_ANGLE_PODIUM_SPEAKER),
        AMP(Constants.Pivot.PIVOT_ANGLE_AMP),
        INTAKEN(Constants.Pivot.PIVOT_ANGLE_INTAKEN),
        STOWED(Constants.Pivot.PIVOT_ANGLE_STOWED),
        OVER_STAGE(Constants.Pivot.PIVOT_ANGLE_OVER_STAGE);

        private final double angle;
        positionState(double angle){

            this.angle = angle;
        }
        public double getValue(){

            return this.angle;
        }
    }

    public void setControlState(controlState pControlState) {
        currentControlState = pControlState;
    }
    public controlState getControlState() {
        return currentControlState;
    }
    public void setPositionState(positionState pPositionState) {
        currentPositionState = pPositionState;
    }
    public positionState getPosititonState() {
        return currentPositionState;
    }
    public void setMovingState(movingState pMovingState) {
        currentMovingState = pMovingState;
    }
    public movingState getMovingState() {
        return currentMovingState;
    }

    //get/set angle from lookuptable
    public void setAutomaticAngleTarget(double automaticAngleTarget) {
        this.automaticAngleTarget = automaticAngleTarget;
    }

    public double getAutomaticAngleTarget() {
        return automaticAngleTarget;
    }

    public double getSetPointMargin() {
        return setPointMargin;
    }

    public void setSetPointMargin(double setPointMargin) {
        this.setPointMargin = setPointMargin;
    }

    public double getAbsoluteAngle() {
        return absoluteAngle;
    }

    public void setAbsoluteAngle(double absoluteAngle) {
        this.absoluteAngle = absoluteAngle;
    }

}
