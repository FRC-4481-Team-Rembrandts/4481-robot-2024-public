package frc.team4481.robot.auto.actions;

import frc.team4481.lib.auto.actions.Action;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.lib.util.CountingDelay;
import frc.team4481.robot.subsystems.Drivetrain;
import frc.team4481.robot.subsystems.DrivetrainManager;
import frc.team4481.robot.subsystems.Outtake;
import frc.team4481.robot.subsystems.OuttakeManager;


public class AimingAutomaticAction implements Action {
    SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
    Drivetrain drivetrain;
    DrivetrainManager drivetrainManager;
    Outtake outtake;
    OuttakeManager outtakeManager;
    boolean isFinished = false;
    private double minDuration;
    private CountingDelay delay;
    public AimingAutomaticAction(){
        drivetrain = (Drivetrain) subsystemHandler.getSubsystemByClass(Drivetrain.class);
        drivetrainManager = drivetrain.getSubsystemManager();
        outtake = (Outtake) subsystemHandler.getSubsystemByClass(Outtake.class);
        outtakeManager = outtake.getSubsystemManager();

        delay = new CountingDelay();

        minDuration = 0;
    }
    public AimingAutomaticAction(double minDuration){
        this();
        this.minDuration = minDuration;
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

        delay.reset();
    }

    @Override
    public void update() {
        //Pass the robot pose from the drivetrain to the outtake
        outtakeManager.setRobotPose(drivetrainManager.getPoseEstimate());
        outtakeManager.setRobotSpeed(drivetrainManager.getCurrentSpeeds());

        isFinished = delay.delay(minDuration) && outtakeManager.getMovingState() == OuttakeManager.movingState.ON_TARGET
                && outtakeManager.isUpdated();

    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void done() {

    }
}
