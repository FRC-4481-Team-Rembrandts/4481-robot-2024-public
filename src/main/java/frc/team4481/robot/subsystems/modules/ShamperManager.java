package frc.team4481.robot.subsystems.modules;

import frc.team4481.lib.subsystems.SubsystemManagerBase;
import frc.team4481.robot.Constants;
import frc.team4481.robot.util.ShamperSpeeds;

public class ShamperManager extends SubsystemManagerBase {

    private controlState currentControlState = controlState.DISABLED;
    private double currentShamperMargin;
    private shootingState currentShootingState = shootingState.PREPARING;
    private velocityState currentVelocityState = velocityState.IDLE;
    private ShamperSpeeds automaticVelocityStates = new ShamperSpeeds(0.0, 0.0, 0.0);
    private boolean motorOverheat = false;
    private boolean shootSensor = false;

    public enum controlState {
        DISABLED,
        AUTOMATIC,
        MANUAL,

    }

    public enum shootingState {
        ON_TARGET,
        PREPARING,

    }


    public enum velocityState {
        AGAINST_SUBWOOFER_FRONT(Constants.Shamper.AGAINST_SUBWOOFER),
        PODIUM(Constants.Shamper.PODIUM),
        AMP(Constants.Shamper.AMP),
        INTAKEN(Constants.Shamper.INTAKEN),
        IDLE(Constants.Shamper.IDLE),
        EJECT(Constants.Shamper.EJECT),
        OVER_STAGE(Constants.Shamper.OVER_STAGE),
        RAMP_AMP(Constants.Shamper.RAMP_AMP),
        RAMP_SPEAKER(Constants.Shamper.RAMP_SPEAKER),
        AUTON_EJECT(Constants.Shamper.AUTON_EJECT);

        private final ShamperSpeeds shamperSpeed;
        velocityState(ShamperSpeeds shamperSpeed){

            this.shamperSpeed = shamperSpeed;
        }
        public ShamperSpeeds getValue(){

            return this.shamperSpeed;
        }

    }

    public void setControlState(controlState pControlState) {currentControlState = pControlState;}
    public controlState getControlState() {return currentControlState;}
    public void setShootingState(shootingState pShootingState) {
        currentShootingState = pShootingState;}
    public shootingState getShootingState() {return currentShootingState;}
    public void setVelocityState(velocityState pVelocityState) {currentVelocityState = pVelocityState;}
    public velocityState getVelocityState() {return currentVelocityState;}

    //From loopuptable
    public ShamperSpeeds getAutomaticVelocityTarget() {
        return automaticVelocityStates;
    }
    public void setAutomaticVelocityTarget(ShamperSpeeds automaticVelocityStates) {
        this.automaticVelocityStates = automaticVelocityStates;
    }

    public double getShamperMargin() {
        return currentShamperMargin;
    }

    public void setShamperMargin(double currentShamperMargin) {
        this.currentShamperMargin = currentShamperMargin;
    }

    public void setMotorOverheat(boolean pOverheat) {
        motorOverheat = pOverheat;
    }
    public boolean getMotorOverheat() {return motorOverheat;}

    public boolean isShootSensor() {
        return shootSensor;
    }

    public void setShootSensor(boolean shootSensor) {
        this.shootSensor = shootSensor;
    }
}
