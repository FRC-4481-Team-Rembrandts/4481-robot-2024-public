package frc.team4481.robot.auto.actions;

import frc.team4481.lib.auto.actions.Action;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.lib.util.CountingDelay;
import frc.team4481.robot.subsystems.*;

/**
 * Kapsalon alles neef
 */
public class AimingAutomaticAndFeedAction implements Action {
    SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
    Drivetrain drivetrain;
    DrivetrainManager drivetrainManager;
    Outtake outtake;
    OuttakeManager outtakeManager;
    Intake intake;
    IntakeManager intakeManager;
    boolean isFinished = false;
    private double feedingTimeout;
    private double aimingTimeout;
    private CountingDelay aimDelay;
    private CountingDelay feedingDelay;
    private CountingDelay minAimTimeDelay;
    public AimingAutomaticAndFeedAction(double aimingTimeout, double feedingTimeout){
        drivetrain = (Drivetrain) subsystemHandler.getSubsystemByClass(Drivetrain.class);
        drivetrainManager = drivetrain.getSubsystemManager();
        outtake = (Outtake) subsystemHandler.getSubsystemByClass(Outtake.class);
        outtakeManager = outtake.getSubsystemManager();
        intake = (Intake) subsystemHandler.getSubsystemByClass(Intake.class);
        intakeManager = intake.getSubsystemManager();

        aimDelay = new CountingDelay();
        feedingDelay = new CountingDelay();
        minAimTimeDelay = new CountingDelay();
        this.feedingTimeout = feedingTimeout;
        this.aimingTimeout = aimingTimeout;
    }
    @Override
    public void start() {
        //Set the outtake to moving and updated false, to make sure it is not on target from a previous setpoint
        outtakeManager.setMovingState(OuttakeManager.movingState.MOVING);
        outtakeManager.setUpdated(false);

        //Pass the robot pose from the drivetrain to the outtake
        outtakeManager.setRobotPose(drivetrainManager.getPoseEstimate());

        //Set the control state to moving
        outtakeManager.setControlState(OuttakeManager.controlState.AUTOMATIC);

        aimDelay.reset();
        minAimTimeDelay.reset();
    }

    @Override
    public void update() {
        //Pass the robot pose from the drivetrain to the outtake
        outtakeManager.setRobotPose(drivetrainManager.getPoseEstimate());
        outtakeManager.setRobotSpeed(drivetrainManager.getCurrentSpeeds());

        if ((outtakeManager.getMovingState() == OuttakeManager.movingState.ON_TARGET
                && outtakeManager.isUpdated() && minAimTimeDelay.delay(0.1))
                || (aimDelay.delay(aimingTimeout))) {
            intakeManager.setControlState(IntakeManager.controlState.TURBO_FEEDER);
        }

        if(intakeManager.getControlState() != IntakeManager.controlState.TURBO_FEEDER){
           feedingDelay.reset();
        } else if (feedingDelay.delay(feedingTimeout)){
            isFinished = true;
        }


    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void done() {
        intakeManager.setControlState(IntakeManager.controlState.DISABLED);

    }
}
