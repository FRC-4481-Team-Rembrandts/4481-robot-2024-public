package frc.team4481.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.team4481.lib.subsystems.SubsystemBase;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.robot.Constants;
import frc.team4481.robot.configuration.ConfigurationHandler;
import frc.team4481.robot.subsystems.modules.Pivot;
import frc.team4481.robot.subsystems.modules.PivotManager;
import frc.team4481.robot.subsystems.modules.Shamper;
import frc.team4481.robot.subsystems.modules.ShamperManager;
import frc.team4481.robot.util.LookupTable;
import frc.team4481.robot.util.LookupTablePreset;
import frc.team4481.robot.util.ScoringHandler;

import java.util.Arrays;
import java.util.List;

import static frc.team4481.robot.Constants.AutoAim.PIVOT_VELOCITY_COMPENSATION_FACTOR;
import static frc.team4481.robot.Constants.Pivot.PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET;


public class Outtake extends SubsystemBase<OuttakeManager> {
    private final SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
    private final ConfigurationHandler configHandler = ConfigurationHandler.getInstance();
    private final ScoringHandler scoringHandler = ScoringHandler.getInstance();
    private final List<SubsystemBase> outtakeModules;
    private Pivot pivot;
    private Shamper shamper;

    private PivotManager pivotManager;
    private ShamperManager shamperManager;

    double distanceToTarget = 0.0;
    double angleSetpoint = 0.0;
    Translation2d targetPose;
    LookupTable lookupTable;

    boolean hasUpdated = false;


    // Mechanism2d
    private Mechanism2d outtakeMechanism;
    private MechanismLigament2d outtakeMechanismShooter;



    public Outtake(){
        name = "Outtake";
        subsystemManager = new OuttakeManager();

        pivot = new Pivot();
        shamper = new Shamper();

        pivotManager = pivot.getSubsystemManager();
        shamperManager = shamper.getSubsystemManager();

        outtakeModules = Arrays.asList(pivot, shamper);

        lookupTable = LookupTable.getInstance();

        // Mechanism2d
        outtakeMechanism = new Mechanism2d(3, 3);
        MechanismRoot2d outtakeMechanismRoot = outtakeMechanism.getRoot("Outtake", 1.311, 0);
        MechanismLigament2d outtakeMechanismSuperStructure = outtakeMechanismRoot.append(
                new MechanismLigament2d("SuperStructure", 0.50, 90, 0, new Color8Bit(Color.kAqua)));
        outtakeMechanismShooter = outtakeMechanismSuperStructure.append(
                new MechanismLigament2d("Shooter", 0.447, -90, 10, new Color8Bit(Color.kWhite)));
    }

    @Override
    public void onStart(double timestamp) {
        if (DriverStation.isTest()){
            subsystemManager.setControlState(OuttakeManager.controlState.DISABLED);
            pivotManager.setControlState(PivotManager.controlState.DISABLED);
        }   else {
            subsystemManager.setControlState(OuttakeManager.controlState.MANUAL);
            pivotManager.setPositionState(PivotManager.positionState.STOWED);
        }
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
            targetPose = Constants.AutoAim.SPEAKER_TARGET_POSE_RED.getTranslation();
        } else {
            targetPose = Constants.AutoAim.SPEAKER_TARGET_POSE_BLUE.getTranslation();
        }

        outtakeModules.forEach(s -> s.onStart(timestamp));

        hasUpdated = false;
    }

    @Override
    public void readPeriodicInputs() {
        outtakeModules.forEach(SubsystemBase::readPeriodicInputs);

        outtakeMechanismShooter.setColor(booleanToColor(shamperManager.isShootSensor()));
        outtakeMechanismShooter.setAngle(pivotManager.getAbsoluteAngle() - 90 - PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET);
    }

    @Override
    public void onLoop(double timestamp) {
        subsystemManager.setMotorOverheat(shamperManager.getMotorOverheat());
        Pose2d correctedPose = compensateRobotPose(subsystemManager.getRobotPose(), subsystemManager.getRobotSpeed());
        //get setpoints from lookupTable
        distanceToTarget = getShortestDistance(correctedPose.getTranslation(), targetPose);
        switch (subsystemManager.getControlState()) {
            case DISABLED -> {
                pivotManager.setControlState(PivotManager.controlState.DISABLED);
                shamperManager.setControlState(ShamperManager.controlState.DISABLED);
            }
            case AUTOMATIC -> {
                pivotManager.setControlState(PivotManager.controlState.AUTOMATIC);
                shamperManager.setControlState(ShamperManager.controlState.AUTOMATIC);

                //Offset the current robot pose based on velocity to compensate for driving while shooting
//                Pose2d correctedPose = compensateRobotPose(subsystemManager.getRobotPose(), subsystemManager.getRobotSpeed());
                //get setpoints from lookupTable
                distanceToTarget = getShortestDistance(correctedPose.getTranslation(), targetPose);
                LookupTablePreset lookupTablePreset = lookupTable.getLookupTablePreset(distanceToTarget);

                //send speeds to shamper
                if (scoringHandler.getScoringPosition() != ScoringHandler.ScoringPosition.LOPJE){
                    shamperManager.setAutomaticVelocityTarget(lookupTablePreset.shamperSpeeds());
                    pivotManager.setAutomaticAngleTarget(lookupTablePreset.pivotAngle());
                } else {
                    shamperManager.setAutomaticVelocityTarget(lookupTablePreset.supercycleSpeeds());
                    pivotManager.setAutomaticAngleTarget(lookupTablePreset.supercycleAngle());
                }
                shamperManager.setShamperMargin(lookupTablePreset.ShamperMargin());
                pivotManager.setSetPointMargin(lookupTablePreset.PivotMargin());

            }
            case MANUAL -> {
                pivotManager.setControlState(PivotManager.controlState.MANUAL);
                shamperManager.setControlState(ShamperManager.controlState.MANUAL);
                pivotManager.setSetPointMargin(Constants.Pivot.MARGIN_SETPOINT_REST);
            }
        }

        switch (subsystemManager.getPositionState()) {
            case INTAKING -> {
                shamperManager.setVelocityState(ShamperManager.velocityState.INTAKEN);
                pivotManager.setPositionState(PivotManager.positionState.INTAKEN);
            }
            case AMP -> {
                shamperManager.setVelocityState(ShamperManager.velocityState.AMP);
                pivotManager.setPositionState(PivotManager.positionState.AMP);
            }
            case PODIUM -> {
                shamperManager.setVelocityState(ShamperManager.velocityState.PODIUM);
                pivotManager.setPositionState(PivotManager.positionState.PODIUM);
            }
            case AGAINST_SUBWOOFER -> {
                shamperManager.setVelocityState(ShamperManager.velocityState.AGAINST_SUBWOOFER_FRONT);
                pivotManager.setPositionState(PivotManager.positionState.AGAINST_SPEAKER);
            }
            case STOWED -> {
                shamperManager.setVelocityState(ShamperManager.velocityState.IDLE);
                pivotManager.setPositionState(PivotManager.positionState.STOWED);
            }
            case EJECT -> {
                shamperManager.setVelocityState(ShamperManager.velocityState.EJECT);
                pivotManager.setPositionState(PivotManager.positionState.STOWED);
            }
            case OVER_STAGE -> {
                shamperManager.setVelocityState(ShamperManager.velocityState.OVER_STAGE);
                pivotManager.setPositionState(PivotManager.positionState.OVER_STAGE);
            }
            case AUTON_EJECT -> {
                shamperManager.setVelocityState(ShamperManager.velocityState.AUTON_EJECT);
                pivotManager.setPositionState(PivotManager.positionState.STOWED);
            }

        }



        //With the states that the manager has set to the sub sub systems, run the on loop
        outtakeModules.forEach(s -> s.onLoop(timestamp));

        //After the sub sub systems have run their on loop, check if they are on target
        if (pivotManager.getMovingState() == PivotManager.movingState.ON_TARGET &&
                shamperManager.getShootingState() == ShamperManager.shootingState.ON_TARGET){

            subsystemManager.setMovingState(OuttakeManager.movingState.ON_TARGET);
        } else {
            subsystemManager.setMovingState(OuttakeManager.movingState.MOVING);
        }

        //After the on target checks have been done, is updated can be set to true.
        //This is done with two variables, to make sure that in the beginning the code makes one more loop
        //before it is actually updated
        //This is necessary because the thread of auto sometimes runs in between lines that are executed here
        if(hasUpdated){
            subsystemManager.setUpdated(true);
        }
        if (!subsystemManager.isUpdated()){
            hasUpdated = true;
        }

        //Smartdashboard is needed exactly here to get the timing right
        SmartDashboard.putBoolean("Outtake/isUpdated", subsystemManager.isUpdated());
        SmartDashboard.putBoolean("Outtake/hasUpdated", hasUpdated);

    }


    @Override
    public void writePeriodicOutputs() {
        outtakeModules.forEach(SubsystemBase::writePeriodicOutputs);
    }

    @Override
    public void onStop(double timestamp) {
        terminate();

        outtakeModules.forEach(s -> s.onStop(timestamp));
    }

    @Override
    public void zeroSensors() {
        outtakeModules.forEach(SubsystemBase::zeroSensors);
    }


    @Override
    public void terminate() {
        outtakeModules.forEach(SubsystemBase::terminate);
    }

    @Override
    public void outputData() {
        SmartDashboard.putString("Outtake/controlState", subsystemManager.getControlState().toString());
        SmartDashboard.putString("Outtake/positionState", subsystemManager.getPositionState().toString());
        SmartDashboard.putString("Outtake/movingState", subsystemManager.getMovingState().toString());
        SmartDashboard.putNumber("Outtake/distance to target", distanceToTarget);

        SmartDashboard.putData("Outtake/mechanism", outtakeMechanism);

        outtakeModules.forEach(SubsystemBase::outputData);
    }

    public double getShortestDistance(Translation2d currentPos, Translation2d target){
        return currentPos.getDistance(target);
    }

    private Pose2d compensateRobotPose(Pose2d robotPose, ChassisSpeeds robotSpeed){
        Translation2d deltaPos;

        deltaPos = new Translation2d(robotSpeed.vxMetersPerSecond*PIVOT_VELOCITY_COMPENSATION_FACTOR, robotSpeed.vyMetersPerSecond*PIVOT_VELOCITY_COMPENSATION_FACTOR);

        Translation2d newTrans = robotPose.getTranslation().plus(deltaPos);
        return new Pose2d(newTrans, robotPose.getRotation());

    }

    private Color8Bit booleanToColor(boolean bool) {
        return bool ? new Color8Bit(Color.kRed) : new Color8Bit(Color.kWhite);
    }

}