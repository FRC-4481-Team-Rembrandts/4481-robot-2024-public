package frc.team4481.robot.auto.actions;

import frc.team4481.lib.auto.actions.SeriesAction;
import frc.team4481.robot.subsystems.OuttakeManager;

public class ScoreManualAction extends SeriesAction {

    public ScoreManualAction(OuttakeManager.positionState manualTarget){
        super(
                new AimingManualAction(manualTarget),
                new StartFeedingAction(0.5)
        );
    }

}
