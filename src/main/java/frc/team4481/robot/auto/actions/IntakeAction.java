package frc.team4481.robot.auto.actions;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.team4481.lib.auto.actions.Action;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.robot.subsystems.Intake;
import frc.team4481.robot.subsystems.IntakeManager;

public class IntakeAction implements Action {
    SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
    Intake intake;
    IntakeManager intakemanager;
    private double startTime;
    private double timeOut;

    private boolean isFinished = false;


    public IntakeAction(double timeOut){
        intake = (Intake) subsystemHandler.getSubsystemByClass(Intake.class);
        intakemanager = intake.getSubsystemManager();
        this.timeOut = timeOut;
    }

    @Override
    public void start() {
        //If the intake is not already holding a note, set the intake to intaking
        if (intakemanager.getControlState() != IntakeManager.controlState.STORE && intakemanager.getControlState() != IntakeManager.controlState.HOLD){
            intakemanager.setControlState(IntakeManager.controlState.INTAKE);
        }
        startTime = MathSharedStore.getTimestamp();

    }

    @Override
    public void update() {
        double timeStamp = MathSharedStore.getTimestamp();
        if (intakemanager.getControlState() == IntakeManager.controlState.HOLD ||
                timeStamp - startTime > timeOut) {
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void done() {
        DataLogManager.log("Intake action done");
    }
}
