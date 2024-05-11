package frc.team4481.robot.auto.actions;

import frc.team4481.lib.auto.actions.ParallelAction;
import frc.team4481.lib.auto.actions.SeriesAction;
import frc.team4481.lib.auto.actions.WaitAction;
import frc.team4481.robot.Constants;
import frc.team4481.robot.subsystems.OuttakeManager;

public class ScoreAutomaticAction extends SeriesAction {

    public ScoreAutomaticAction(){
        super(
                new ParallelAction(new AimingAutomaticAction(),
                        new TurnToPoseAction(1)
                ),
                new StartFeedingAction(0.7)

        );
    }

}
