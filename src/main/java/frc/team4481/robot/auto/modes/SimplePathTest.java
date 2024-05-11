package frc.team4481.robot.auto.modes;

import edu.wpi.first.wpilibj.DataLogManager;
import frc.team4481.lib.auto.actions.ParallelAction;
import frc.team4481.lib.auto.actions.SeriesAction;
import frc.team4481.lib.auto.mode.AutoModeBase;
import frc.team4481.lib.auto.mode.AutoModeEndedException;
import frc.team4481.lib.path.PathNotLoadedException;
import frc.team4481.lib.path.TrajectoryHandler;
import frc.team4481.robot.auto.actions.*;
import frc.team4481.robot.auto.selector.AutoMode;
import frc.team4481.robot.auto.selector.Disabled;
import frc.team4481.robot.subsystems.OuttakeManager;

@Disabled
@AutoMode(displayName = "[Test] SimplePathTest")
public class SimplePathTest extends AutoModeBase {
    String path_1 = "New Path";

    @Override
    protected void initialize() throws AutoModeEndedException, PathNotLoadedException {
        TrajectoryHandler trajectoryHandler = TrajectoryHandler.getInstance();
        try {
            trajectoryHandler.setUnloadedPath(path_1);
        } catch (PathNotLoadedException e) {
            e.printStackTrace();
        }
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new SetInitalPositionAction(path_1));
        runAction(new SeriesAction(
                new DriveAndShootTimedAction(path_1, new double[]{0.90}),
                new AimingManualAction(OuttakeManager.positionState.STOWED)
        ));
    }
}
