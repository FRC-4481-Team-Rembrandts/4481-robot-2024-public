package frc.team4481.lib.looper;

import frc.team4481.lib.subsystems.SubsystemBase;
import frc.team4481.lib.subsystems.SubsystemHandler;

import java.util.List;

/**
 * Base loop for execution of the loop functions inside each subsystem when the robot is disabled.
 */
public class DisabledSuperSubsystemLoop implements Loop {
    private final List<SubsystemBase> allSubsystems;

    public DisabledSuperSubsystemLoop(SubsystemHandler mSubsystemManger){
        this.allSubsystems = mSubsystemManger.getSubsystems();
    }

    @Override
    public void onStart(double timestamp) {
    }

    @Override
    public void onLoop(double timestamp) {
        allSubsystems.forEach(SubsystemBase::readPeriodicInputs);
        
        try {
            allSubsystems.forEach(SubsystemBase::outputData);
        } catch (Exception e) {
            System.out.println("Something is not initialized in disabled in outputData");
            e.printStackTrace();
        }
    }

    @Override
    public void onStop(double timestamp) {
    }
}
