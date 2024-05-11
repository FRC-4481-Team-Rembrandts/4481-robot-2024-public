package frc.team4481.lib.subsystems;

import frc.team4481.lib.looper.Loop;

import java.util.UUID;

public abstract class SubsystemBase<TController> extends SubsystemManagerBase implements Loop {
    public String name = "Unnamed Subsystem";
    UUID id = null;

    protected TController subsystemManager;

    public abstract void readPeriodicInputs();

    public abstract void writePeriodicOutputs();

    public abstract void zeroSensors();

    public abstract void terminate();

    public abstract void outputData();

    public TController getSubsystemManager(){
        return subsystemManager;
    }
}
