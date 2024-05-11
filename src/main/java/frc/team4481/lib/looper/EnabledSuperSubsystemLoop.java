package frc.team4481.lib.looper;

import frc.team4481.lib.subsystems.SubsystemBase;
import frc.team4481.lib.subsystems.SubsystemHandler;

import java.util.List;

/**
 * Base loop for execution of the loop functions inside each subsystem when the robot is enabled.
 */
public class EnabledSuperSubsystemLoop implements Loop {
    private final List<SubsystemBase> allSubsystems;

    // TODO change manager to handler
    public EnabledSuperSubsystemLoop(SubsystemHandler mSubsystemManger){
        this.allSubsystems = mSubsystemManger.getSubsystems();
    }

    @Override
    public void onStart(double timestamp) {
        allSubsystems.forEach(s -> s.onStart(timestamp));
    }

    @Override
    public void onLoop(double timestamp) {
        allSubsystems.forEach(SubsystemBase::readPeriodicInputs);
        allSubsystems.forEach(s -> s.onLoop(timestamp));
        allSubsystems.forEach(SubsystemBase::writePeriodicOutputs);
        allSubsystems.forEach(SubsystemBase::outputData);
    }

    @Override
    public void onStop(double timestamp) {
        allSubsystems.forEach(s -> s.onStop(timestamp));
    }
}
