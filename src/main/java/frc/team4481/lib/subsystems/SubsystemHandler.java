package frc.team4481.lib.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class SubsystemHandler {
    public static SubsystemHandler instance = null;

    private List<SubsystemBase> subsystemList;

    private SubsystemHandler(){}

    public static SubsystemHandler getInstance(){
        if(instance == null){
            instance = new SubsystemHandler();
        }

        return instance;
    }

    public void setSubsystems(SubsystemBase... subsystems) {
        subsystemList = Arrays.asList(subsystems);
    }

    public List<SubsystemBase> getSubsystems() {
        return subsystemList;
    }

    public SubsystemBase getSubsystemByName(String name){
        return subsystemList.stream().filter(subsystemBase -> name.equals(subsystemBase.name)).findFirst().orElse(null);
    }
    public SubsystemBase getSubsystemByClass(Class subsystem){
        return subsystemList.stream().filter(subsystemBase -> subsystem.equals(subsystemBase.getClass())).findFirst().orElse(null);
    }
    public SubsystemManagerBase getSubsystemControllerByClass(Class controller){
        return subsystemList.stream().filter(subsystemBase -> controller.equals(subsystemBase.subsystemManager)).findFirst().orElse(null);
    }

    public List<String> getSubsystemNames(){
        List<String> names = new ArrayList<>();
        subsystemList.forEach(subsystemBase -> names.add(subsystemBase.name));
        return names;
    }

    public void outputToSmartDashboard() {
        subsystemList.forEach(SubsystemBase::outputData);
    }

    public void terminateAll() {
        subsystemList.forEach(SubsystemBase::terminate);
    }

}
