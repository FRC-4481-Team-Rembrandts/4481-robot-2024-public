package frc.team4481.robot.auto.modes;

import frc.team4481.lib.auto.actions.ParallelAction;
import frc.team4481.lib.auto.actions.SeriesAction;
import frc.team4481.lib.auto.mode.AutoModeBase;
import frc.team4481.lib.auto.mode.AutoModeEndedException;
import frc.team4481.lib.path.PathNotLoadedException;
import frc.team4481.lib.path.TrajectoryHandler;
import frc.team4481.robot.auto.actions.*;
import frc.team4481.robot.auto.selector.AutoMode;
import frc.team4481.robot.subsystems.Intake;
import frc.team4481.robot.subsystems.IntakeManager;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.robot.subsystems.OuttakeManager;


@AutoMode(displayName = "[Top] 5_note_runner")
public class Top_5_note_runner_Auto extends AutoModeBase {
    String path_1 = "Start_Cn4";
    String path_2 = "Shooting_CN4";
    String path_3 = "CN3_pick_stage";
    String path_4 = "Cn4_nonote";
    String path_5 = "Shooting_CN3";
    String path_6 = "CN3_nonote";
    String path_7 = "Shooting_CN5";
   // String path_8 = "FN3_shooting";
    String path_9 = "Midline_Cleanup";
    private Intake intake;
    private IntakeManager intakeManager;
    private final SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();


    @Override
    protected void initialize() throws AutoModeEndedException, PathNotLoadedException {
        intake = (Intake) subsystemHandler.getSubsystemByClass(Intake.class);
        intakeManager = intake.getSubsystemManager();
        TrajectoryHandler trajectoryHandler = TrajectoryHandler.getInstance();
        try {
            trajectoryHandler.setUnloadedPath(path_1);
            trajectoryHandler.setUnloadedPath(path_2);
            trajectoryHandler.setUnloadedPath(path_3);
            trajectoryHandler.setUnloadedPath(path_4);
            trajectoryHandler.setUnloadedPath(path_5);
            trajectoryHandler.setUnloadedPath(path_6);
            trajectoryHandler.setUnloadedPath(path_7);
      //      trajectoryHandler.setUnloadedPath(path_8);
            trajectoryHandler.setUnloadedPath(path_9);
        } catch (PathNotLoadedException e) {
            e.printStackTrace();
        }
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new SetInitalPositionAction(path_1));
        runAction(new ParallelAction(
                new DrivePathAction(path_1),
                new SeriesAction(
                        new AimingAutomaticAndFeedAction(0.6, 1.5),
                        new IntakeAction(1.3)
                )
        ));
        if (intakeManager.getControlState() == IntakeManager.controlState.STORE || intakeManager.getControlState() == IntakeManager.controlState.HOLD) {
            runAction(new DriveAndShootTimedAction(path_2, new double[]{1.53}));
            runAction(new ParallelAction(
                    new DrivePathAction(path_3),
                    new AimingManualAction(OuttakeManager.positionState.STOWED),
                    new IntakeAction(1.80)
            ));
        }else{
            runAction(new ParallelAction(
                    new DrivePathAction(path_4),
                    new IntakeAction(1.40)
            ));
        }
        if(intakeManager.getControlState() == IntakeManager.controlState.STORE  || intakeManager.getControlState() == IntakeManager.controlState.HOLD){
            runAction(new DriveAndShootTimedAction(path_5, new double[]{2.09}));
        }else{
            runAction(new ParallelAction(
                    new DrivePathAction(path_6),
                    new IntakeAction(1.84)
            ));
        }
        runAction(new DriveAndShootTimedAction(path_7, new double[]{1.73
        }));
//        runAction(new ParallelAction(
//                new DrivePathAction(path_8),
//                new IntakeAction(1.96)
//        ));
//        runAction(new ScoreAutomaticAction());
        runAction(new ParallelAction(
                new DrivePathAction(path_9),
                new IntakeAction(9.0)
        ));
    }
}