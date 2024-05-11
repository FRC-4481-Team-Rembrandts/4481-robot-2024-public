package frc.team4481.robot.auto.actions;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.team4481.lib.auto.actions.Action;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.robot.Constants;
import frc.team4481.robot.autoaim.TurnToPose;
import frc.team4481.robot.subsystems.Drivetrain;
import frc.team4481.robot.subsystems.DrivetrainManager;

import static frc.team4481.robot.Constants.AutoAim.ROTATION_VELOCITY_COMPENSATION_FACTOR;
import static java.lang.Math.abs;

public class TurnToPoseAction implements Action {
    private Pose2d targetSpeaker;
    TurnToPose turnToPose;
    ChassisSpeeds desiredSpeed = new ChassisSpeeds();
    Drivetrain drivertrain;
    DrivetrainManager drivetrainManager;
    private double startTime;
    private boolean isFinished = false;
    private double timeOut;

    public TurnToPoseAction(double timeOut){
        drivertrain = (Drivetrain) SubsystemHandler.getInstance().getSubsystemByClass(Drivetrain.class);
        drivetrainManager = drivertrain.getSubsystemManager();

        this.timeOut = timeOut;

    }


    @Override
    public void start() {
        //Enable the drivetrain, altijd handig
        drivetrainManager.setControlState(DrivetrainManager.controlState.ENABLED);

        startTime = MathSharedStore.getTimestamp();
        turnToPose = new TurnToPose(Constants.AutoAim.AUTO_AIM_MARGIN,Constants.AutoAim.AUTO_AIM_SETPOINT_MARGIN,Constants.AutoAim.AUTO_AIM_KP,Constants.AutoAim.AUTO_AIM_KD, ROTATION_VELOCITY_COMPENSATION_FACTOR);

        //Select the target based on alliance color
        if (DriverStation.getAlliance().isPresent() &&
                DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            targetSpeaker = Constants.AutoAim.SPEAKER_TARGET_POSE_RED;
        } else {
            targetSpeaker = Constants.AutoAim.SPEAKER_TARGET_POSE_BLUE;
        }


    }

    @Override
    public void update() {
        double timeStamp = MathSharedStore.getTimestamp();

        desiredSpeed = turnToPose.getTargetSpeeds(drivetrainManager.getPoseEstimate(),targetSpeaker);
        drivetrainManager.setDesiredSpeeds(desiredSpeed);

        if (turnToPose.isInPidMargin() || timeStamp - startTime > timeOut){
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void done() {
        drivetrainManager.setControlState(DrivetrainManager.controlState.DISABLED);
    }
}
