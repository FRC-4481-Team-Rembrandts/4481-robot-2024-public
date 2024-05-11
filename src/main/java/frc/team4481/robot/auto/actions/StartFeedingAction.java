package frc.team4481.robot.auto.actions;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.MathSharedStore;
import frc.team4481.lib.auto.actions.Action;
import frc.team4481.lib.path.AdaptiveTrajectoryTimeSampler;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.lib.util.CountingDelay;
import frc.team4481.robot.subsystems.Intake;
import frc.team4481.robot.subsystems.IntakeManager;
import frc.team4481.robot.subsystems.Outtake;
import frc.team4481.robot.subsystems.OuttakeManager;

public class StartFeedingAction implements Action {
    SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
    Outtake outtake;
    OuttakeManager outtakemanager;
    Intake intake;
    IntakeManager intakemanager;
    private double startTime;
    private boolean isFinished = false;

    private double timeOut;

    /**
     *
     * @param timeOut the delay in seconds after the action should be canceled
     */
    public StartFeedingAction(double timeOut){
        intake = (Intake) subsystemHandler.getSubsystemByClass(Intake.class);
        intakemanager = intake.getSubsystemManager();
        outtake = (Outtake) subsystemHandler.getSubsystemByClass(Outtake.class);
        outtakemanager = outtake.getSubsystemManager();

        this.timeOut = timeOut;
    }

    @Override
    public void start() {
        startTime = MathSharedStore.getTimestamp();

        if (outtakemanager.getMovingState() == OuttakeManager.movingState.ON_TARGET){
            intakemanager.setControlState(IntakeManager.controlState.INTAKE_FEEDER);
        }
    }

    @Override
    public void update() {
        double timeStamp = MathSharedStore.getTimestamp();

        if (intakemanager.getControlState() != IntakeManager.controlState.INTAKE_FEEDER && outtakemanager.getMovingState() == OuttakeManager.movingState.ON_TARGET){
            intakemanager.setControlState(IntakeManager.controlState.INTAKE_FEEDER);
        }
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
    intakemanager.setControlState(IntakeManager.controlState.DISABLED);

    }
}
