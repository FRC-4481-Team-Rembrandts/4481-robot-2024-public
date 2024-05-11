package frc.team4481.robot.subsystems;

import frc.team4481.lib.subsystems.SubsystemManagerBase;

public class IntakeManager extends SubsystemManagerBase {

    private boolean sensorHigh = false;
    private boolean sensorLow = false;

    private controlState currentControlState = controlState.DISABLED;
    private boolean storageOccupied = false;

    public enum controlState {
        DISABLED,
        INTAKE,
        REVERSE_INTAKE,
        HOLD,
        STORE,
        INTAKE_FEEDER,
        SOURCE_INTAKE,
        TURBO_FEEDER

    }


    public boolean isSensorLow() {
        return sensorLow;
    }

    public void setSensorLow(boolean sensorLow) {
        this.sensorLow = sensorLow;
    }

    public boolean isSensorHigh() {
        return sensorHigh;
    }

    public void setSensorHigh(boolean sensorHigh) {
        this.sensorHigh = sensorHigh;
    }


    public void setControlState(controlState pControlState) {
        currentControlState = pControlState;
    }
    public controlState getControlState() {
        return currentControlState;
    }

    public boolean isStorageOccupied() {
        return storageOccupied;
    }

    public void setStorageOccupied(boolean storageOccupied) {
        this.storageOccupied = storageOccupied;
    }
}
