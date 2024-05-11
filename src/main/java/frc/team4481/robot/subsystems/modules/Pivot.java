package frc.team4481.robot.subsystems.modules;


import com.revrobotics.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.lib.hardware.LazyCANSparkMax;
import frc.team4481.lib.subsystems.SubsystemBase;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.lib.util.CountingDelay;
import frc.team4481.robot.Constants;
import frc.team4481.robot.configuration.ConfigurationHandler;

import static frc.team4481.robot.Constants.Pivot.*;
import static frc.team4481.robot.Constants.kLooperDt;

public class Pivot extends SubsystemBase<PivotManager> {
    private final SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
    private final ConfigurationHandler configHandler = ConfigurationHandler.getInstance();
    private LazyCANSparkMax pivot;
    private RelativeEncoder pivotEncoder;
    private SparkPIDController pivotPID;
    private AbsoluteEncoder absoluteEncoder;
    private TrapezoidProfile.Constraints profileConstraints;
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setPoint;
    private CountingDelay tempratureCount;


    private double targetAngle = PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET;

    private double prevTargetVelocity = 0;
    private double previousT = 0;
    private double absolutePos = 0;
    private boolean isOverHeat = false;



    public Pivot(){
        name = "Pivot";
        subsystemManager = new PivotManager();

        pivot = new LazyCANSparkMax(Constants.HardwareMap.PIVOT_CAN_ID, CANSparkLowLevel.MotorType.kBrushless);
        pivot.setIdleMode(CANSparkBase.IdleMode.kBrake);
        pivot.setSmartCurrentLimit(CURRENT_LIMIT_PIVOT); // TODO test on practice field
        pivotEncoder = pivot.getEncoder();
        pivotEncoder.setPositionConversionFactor(Constants.Pivot.PIVOT_POSITION_CONVERSION_FACTOR * Constants.Pivot.PIVOT_GEAR_RATIO);
        pivotEncoder.setVelocityConversionFactor(Constants.Pivot.PIVOT_GEAR_RATIO * PIVOT_REL_ENC_VEL_CONVERSION_FACTOR);
        pivot.setInverted(true);


        
        absoluteEncoder = pivot.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        absoluteEncoder.setInverted(true);
        absoluteEncoder.setPositionConversionFactor(Constants.Pivot.PIVOT_POSITION_CONVERSION_FACTOR);
        absoluteEncoder.setVelocityConversionFactor(Constants.Pivot.PIVOT_ABS_ENC_VEL_CONVERSION_FACTOR);
        absoluteEncoder.setAverageDepth(2);
        absoluteEncoder.setZeroOffset(PIVOT_ABS_ENC_ZERO_OFFSET);
        pivot.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 10);

//                .setPeriodicFrameRate(CANSparkLowLevel.PeriodicFrame.kStatus4, 10);
        pivotPID = pivot.getPIDController();
        pivotPID.setP(Constants.Pivot.PIVOT_KP_AUTOMATIC,1);
        pivotPID.setP(Constants.Pivot.PIVOT_KP,0);
        pivotPID.setFeedbackDevice(absoluteEncoder);

        pivot.burnFlash();

        profileConstraints = new TrapezoidProfile.Constraints(MAX_VEL, MAX_ACCEL);

        tempratureCount = new CountingDelay();
        tempratureCount.reset();
    }

    @Override
    public void onStart(double timestamp) {
        if (DriverStation.isAutonomous()){
            subsystemManager.setControlState(PivotManager.controlState.DISABLED);
        } else {
            subsystemManager.setControlState(PivotManager.controlState.DISABLED);
        }
        double holdingAngle = absoluteEncoder.getPosition();
        setPoint = new TrapezoidProfile.State(holdingAngle, 0);
        goal = new TrapezoidProfile.State(holdingAngle, 0);

        previousT = timestamp;
        zeroSensors();

    }

    @Override
    public void readPeriodicInputs() {
        absolutePos = absoluteEncoder.getPosition();

        subsystemManager.setAbsoluteAngle(absolutePos);
    }

    @Override
    public void onLoop(double timestamp) {
        switch (subsystemManager.getControlState()) {
            case DISABLED:
//              do nothing
                pivot.set(0);
                break;

            case AUTOMATIC:
//                for angle from lookup table

                    targetAngle = subsystemManager.getAutomaticAngleTarget();
                    pivotTargetPID(targetAngle);

                break;
            case MANUAL:

                    targetAngle = subsystemManager.getPosititonState().getValue();
                    pivotTarget(targetAngle, timestamp);

                break;
        }


        //when the pivot is not in position; movingState needs to be in MOVING
        double pivotMargin = subsystemManager.getSetPointMargin();
        //For amping, the pivot is in a deadspot where it traverses the play, so if the pivot is in amp mode, increase the margin
        if (subsystemManager.getControlState() == PivotManager.controlState.MANUAL &&
                subsystemManager.getPosititonState() == PivotManager.positionState.AMP) {
            pivotMargin = MARGIN_SETPOINT_AMPING;
        }

        //Check if the pivot is on target
        if (isOnTargetAngle(targetAngle, absolutePos, pivotMargin)){
            subsystemManager.setMovingState(PivotManager.movingState.ON_TARGET);
        } else {
            subsystemManager.setMovingState(PivotManager.movingState.MOVING);
        }

    }

    public boolean isOnTargetAngle(double desiredAngle, double currentAngle, double margin){
        return Math.abs(currentAngle-desiredAngle) <= margin;
    }

    @Override
    public void writePeriodicOutputs() {
        SmartDashboard.putNumber("Outtake/Pivot/target velocity", setPoint.velocity);
        SmartDashboard.putNumber("Outtake/Pivot/target position", setPoint.position);
    }

    @Override
    public void onStop(double timestamp) {
        terminate();
    }

    @Override
    public void zeroSensors() {
        prevTargetVelocity = 0;
    }


    @Override
    public void terminate() {
        pivotPID.setReference(0.0, CANSparkBase.ControlType.kDutyCycle, 1);
    }

    @Override
    public void outputData() {

        SmartDashboard.putString("Outtake/Pivot/controlState", subsystemManager.getControlState().toString());
        SmartDashboard.putString("Outtake/Pivot/positionState", subsystemManager.getPosititonState().toString());
        SmartDashboard.putString("Outtake/Pivot/movingState", subsystemManager.getMovingState().toString());
        SmartDashboard.putNumber("Outtake/Pivot/target angle", targetAngle);
        SmartDashboard.putNumber("Outtake/Pivot/current position", absolutePos);
        SmartDashboard.putNumber("Outtake/Pivot/current rel enc pos", pivotEncoder.getPosition());
        SmartDashboard.putNumber("Outtake/Pivot/current velocity", absoluteEncoder.getVelocity());

        SmartDashboard.putNumber("Outtake/Pivot/error angle", absolutePos - targetAngle);

        SmartDashboard.putNumber("Current/pivot", pivot.getOutputCurrent());
        SmartDashboard.putBoolean("Outtake/Pivot/overheat", isOverHeat);
    }


    private void pivotTarget(double desiredAngle, double timestamp) {
        if (absolutePos < Constants.Pivot.DEADZONE_PIVOT_HORIZONTAL + PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET && desiredAngle  < Constants.Pivot.DEADZONE_PIVOT_HORIZONTAL+ PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET){
            pivotPID.setReference(0, CANSparkBase.ControlType.kDutyCycle, 2);
            return;
        }


        goal = new TrapezoidProfile.State(desiredAngle, 0);

        double dt = timestamp - previousT;

        double targetPosition;
        double targetVelocity;
        double targetAcceleration;

        //Check if arm is close enough to setpoint
        double error = desiredAngle - absoluteEncoder.getPosition();
        if (Math.abs(error) <= MOTION_PROFILE_DISABLE_MARGIN) {
            targetVelocity = 0;
            targetAcceleration = 0;
            targetPosition = desiredAngle;
        } else {
            // Create a motion profile with the given maximum velocity and maximum
            // acceleration constraints for the next setpoint, the desired goal, and the
            // current setpoint.
            TrapezoidProfile profile = new TrapezoidProfile(profileConstraints);
            setPoint = profile.calculate(kLooperDt,setPoint,goal);
            targetPosition = setPoint.position;
            targetVelocity = setPoint.velocity;
            targetAcceleration = (targetVelocity - prevTargetVelocity) / dt;
        }

        // Arm Feed Forward
        double arbFFComponent = KS
                + KG * Math.cos(Math.toRadians(absolutePos- PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET))
                + KV * targetVelocity
                + KA * targetAcceleration;

        pivotPID.setReference(targetPosition, CANSparkMax.ControlType.kPosition, 0, arbFFComponent, SparkPIDController.ArbFFUnits.kVoltage);

        prevTargetVelocity = targetVelocity;
        previousT = timestamp;

    }

    private void pivotTargetPID(double desiredAngle){
        resetMotionProfile();


        // turned off beun slew rate
//        if (desiredAngle - absolutePos >= 25) {
//            desiredAngle = absolutePos + 20;
//        }

        // Arm Feed Forward
        double arbFFComponent = KS
                + KG * Math.cos(Math.toRadians(absolutePos- PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET));


        pivotPID.setReference(desiredAngle, CANSparkMax.ControlType.kPosition, 1, arbFFComponent, SparkPIDController.ArbFFUnits.kVoltage);
    }

    private void resetMotionProfile(){
        //reset setpoint
        setPoint.position = absolutePos;
        setPoint.velocity = 0;
    }

//    private boolean pivotOverheatCheck(){
//        if (tempratureCount.delay(PIVOT_HEAT_TIME_DELAY)) {
//            tempratureCount.reset();
//            isOverHeat = pivot.getMotorTemperature() > PIVOT_HEAT_LIMIT;
//        }
//        return isOverHeat;
//    }

}