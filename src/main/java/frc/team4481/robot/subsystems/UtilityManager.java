package frc.team4481.robot.subsystems;

import frc.team4481.lib.subsystems.SubsystemManagerBase;

public class UtilityManager extends SubsystemManagerBase {
    private controlState currentControlState = controlState.DISABLED;

    private boolean isNoteInStorage = false;





    public boolean isNoteInStorage() {
        return isNoteInStorage;
    }

    public void setNoteInStorage(boolean noteInStorage) {
        isNoteInStorage = noteInStorage;
    }





    public enum controlState {
        DISABLED,
        ENABLED,
    }

    public void setControlState(controlState pControlState) {
        currentControlState = pControlState;
    }

    public controlState getControlState() {
        return currentControlState;
    }
}