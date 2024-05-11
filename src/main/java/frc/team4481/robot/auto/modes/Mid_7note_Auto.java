package frc.team4481.robot.auto.modes;

import frc.team4481.lib.auto.actions.ParallelAction;
import frc.team4481.lib.auto.mode.AutoModeBase;
import frc.team4481.lib.auto.mode.AutoModeEndedException;
import frc.team4481.lib.path.PathNotLoadedException;
import frc.team4481.lib.path.TrajectoryHandler;
import frc.team4481.robot.auto.actions.*;
import frc.team4481.robot.auto.selector.AutoMode;

@AutoMode(displayName = "[Mid] 7 note Auto")
public class Mid_7note_Auto extends AutoModeBase {
    String path_1 = "FN123_7.5_note";
    String path_2 = "Improve_CN5.2";



    @Override
    protected void initialize() throws AutoModeEndedException, PathNotLoadedException {
        TrajectoryHandler trajectoryHandler = TrajectoryHandler.getInstance();
        try {
            trajectoryHandler.setUnloadedPath(path_1);
            trajectoryHandler.setUnloadedPath(path_2);
        } catch (PathNotLoadedException e) {
            e.printStackTrace();
        }
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new SetInitalPositionAction(path_1));
        runAction(
                new ParallelAction(
                        new AimingAutomaticAndFeedAction(0.6, 3.3),
                        new DrivePathAction(path_1)
                )
        );

        runAction(new DriveAndShootTimedAction(path_2,new double[]{3.1, 7.15, 10.6}));
    }
}