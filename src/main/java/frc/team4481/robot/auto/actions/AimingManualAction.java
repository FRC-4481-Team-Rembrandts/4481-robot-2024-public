package frc.team4481.robot.auto.actions;

import frc.team4481.lib.auto.actions.Action;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.robot.subsystems.Outtake;
import frc.team4481.robot.subsystems.OuttakeManager;

public class AimingManualAction implements Action {
    SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
    Outtake outtake;
    OuttakeManager outtakeManager;
    boolean isFinished = false;

    OuttakeManager.positionState manualTarget;





    public AimingManualAction(OuttakeManager.positionState manualTarget){
        outtake = (Outtake) subsystemHandler.getSubsystemByClass(Outtake.class);
        outtakeManager = outtake.getSubsystemManager();

        this.manualTarget = manualTarget;
    }
    @Override
    public void start() {
        //Set the outtake to moving and updated false, to make sure it is not on target from a previous setpoint
        outtakeManager.setMovingState(OuttakeManager.movingState.MOVING);
        outtakeManager.setUpdated(false);

        //Set the target of the outtake
        outtakeManager.setPositionState(manualTarget);
        outtakeManager.setControlState(OuttakeManager.controlState.MANUAL);

    }

    @Override
    public void update() {
        //Only finish when the outtake is on target and has updated
        isFinished = outtakeManager.getMovingState() == OuttakeManager.movingState.ON_TARGET
                    && outtakeManager.isUpdated();

    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void done() {

    }
}
