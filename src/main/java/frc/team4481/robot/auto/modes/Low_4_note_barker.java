package frc.team4481.robot.auto.modes;

import frc.team4481.lib.auto.actions.ParallelAction;
import frc.team4481.lib.auto.mode.AutoModeBase;
import frc.team4481.lib.auto.mode.AutoModeEndedException;
import frc.team4481.lib.path.PathNotLoadedException;
import frc.team4481.lib.path.TrajectoryHandler;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.robot.auto.actions.*;
import frc.team4481.robot.auto.selector.AutoMode;
import frc.team4481.robot.auto.selector.Disabled;
import frc.team4481.robot.subsystems.Intake;
import frc.team4481.robot.subsystems.IntakeManager;
import frc.team4481.robot.subsystems.OuttakeManager;

@Disabled
@AutoMode(displayName = "[Bottom] 4 note Q90")
public class Low_4_note_barker extends AutoModeBase {
    String path_1 = "LOW_CN1";
    String path_2 = "SHOOT_CN12";
    String path_3 = "Recover_CN12";
    String path_4 = "SHOOT_CN2";
    String path_5 = "Recover_CN23";
    String path_6 = "CN3_understage";
    String path_7 = "SHOOT_CN3";
    String path_8 = "FN1_4_note_runner";
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
            trajectoryHandler.setUnloadedPath(path_8);
        } catch (PathNotLoadedException e) {
            e.printStackTrace();
        }
    }
    @Override
    protected void routine() throws AutoModeEndedException {
        //shoot
//        runAction(new SetInitalPositionAction(path_1));
//        runAction(new DriveAndShootTimedAction(path_1, new double[]{0.92, 4.14, 7.07}));
//        runAction(new ParallelAction(
//                        new DrivePathAction(path_2),
//                        new ParallelAction(
//                                new AimingManualAction(OuttakeManager.positionState.STOWED)),
//                                new IntakeAction(2.9))
//        );
//        runAction(new DriveAndShootTimedAction(path_3, new double[]{0.82}));

        runAction(new SetInitalPositionAction(path_1));
        runAction(new DriveAndShootTimedAction(path_1, new double[] {0.7}));
        if (intakeManager.getControlState() == IntakeManager.controlState.STORE || intakeManager.getControlState() == IntakeManager.controlState.HOLD ) {
            runAction(new DriveAndShootTimedAction(path_2, new double [] {2}));
        } else {
            runAction(new DrivePathAction(path_3));
        }

//        if (intakeManager.getControlState() == IntakeManager.controlState.STORE || intakeManager.getControlState() == IntakeManager.controlState.HOLD) {
            runAction(new DriveAndShootTimedAction(path_4, new double[] {2.1-0.15}));
//            runAction(new ParallelAction(
//                    new AimingManualAction(OuttakeManager.positionState.STOWED),
//                    new IntakeAction(2.25),
//                    new DrivePathAction(path_6) )
////            );
//        } else {
//            runAction(new DrivePathAction(path_5));
//            runAction(new AimingManualAction(OuttakeManager.positionState.STOWED));
//        }

  //      runAction(new DrivePathAction(path_7));
  //      runAction(new ScoreAutomaticAction());

        runAction(new DriveAndShootTimedAction(path_8, new double[]{0.10})); //dit werkt, 0.10 doet eigenlijk niks
        runAction(new ScoreAutomaticAction());


    }
}
