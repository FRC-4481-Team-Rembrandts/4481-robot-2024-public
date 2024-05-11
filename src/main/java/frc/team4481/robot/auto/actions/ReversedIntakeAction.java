package frc.team4481.robot.auto.actions;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.team4481.lib.auto.actions.Action;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.robot.subsystems.Intake;
import frc.team4481.robot.subsystems.IntakeManager;

public class ReversedIntakeAction implements Action {
    SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
    Intake intake;
    IntakeManager intakemanager;
    private double startTime;
    private double timeOut;

    private boolean isFinished = false;


    public ReversedIntakeAction(double timeOut){
        intake = (Intake) subsystemHandler.getSubsystemByClass(Intake.class);
        intakemanager = intake.getSubsystemManager();
        this.timeOut = timeOut;
    }

    @Override
    public void start() {
        intakemanager.setControlState(IntakeManager.controlState.REVERSE_INTAKE);
        startTime = MathSharedStore.getTimestamp();
    }

    @Override
    public void update() {
        double timeStamp = MathSharedStore.getTimestamp();
        if (timeStamp - startTime > timeOut) {
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void done() {
        intakemanager.setControlState(IntakeManager.controlState.DISABLED);// ekkes kijken of dit wel moet?
        DataLogManager.log("Reversed intake action done");
    }
}
