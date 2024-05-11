package frc.team4481.robot.subsystems.modules;


import com.revrobotics.*;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.lib.hardware.LazyCANSparkFlex;
import frc.team4481.lib.subsystems.SubsystemBase;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.lib.util.CountingDelay;
import frc.team4481.robot.Constants;
import frc.team4481.robot.configuration.ConfigurationHandler;
import frc.team4481.robot.util.ShamperSpeeds;

import static frc.team4481.robot.Constants.Shamper.*;

public class Shamper extends SubsystemBase<ShamperManager> {
    private final SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
    private final ConfigurationHandler configHandler = ConfigurationHandler.getInstance();
    private LazyCANSparkFlex kickerMotor;
    private RelativeEncoder kickerEncoder;
    private SparkPIDController kickerPID;
//    private LazyCANSparkBase topShooter;
    private LazyCANSparkFlex topShooter;
    private RelativeEncoder topEncoder;
    private SparkPIDController topPID;
//    private LazyCANSparkBase bottomShooter;
    private LazyCANSparkFlex bottomShooter;
    private RelativeEncoder bottomEncoder;
    private SparkPIDController bottomPID;
    private ShamperSpeeds targetSpeeds;
    private CountingDelay motorTemperatureCount;
    private DigitalInput shootSensor;


    public Shamper(){
        name = "Shamper";
        subsystemManager = new ShamperManager();

        kickerMotor = new LazyCANSparkFlex(
                Constants.HardwareMap.KICKER_SHOOTER_CAN_ID,
                CANSparkLowLevel.MotorType.kBrushless
        );
        kickerMotor.setSmartCurrentLimit(CURRENT_LIMIT_KICKER);
        kickerMotor.setInverted(true);
        kickerMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        kickerEncoder = kickerMotor.getEncoder();
        kickerEncoder.setVelocityConversionFactor(Constants.Shamper.CONVERSIONFACTOR_KICKER);
        kickerPID = kickerMotor.getPIDController();
        kickerPID.setP(Constants.Shamper.KICKER_KP,0);
        kickerPID.setI(Constants.Shamper.KICKER_KI,0);
        kickerPID.setIZone(Constants.Shamper.SHAMPER_KI_DEADZONE, 0);
        kickerPID.setIMaxAccum(Constants.Shamper.SHAMPER_KI_MAX_ACCUM, 0);
        kickerMotor.burnFlash();


        topShooter = new LazyCANSparkFlex(
                Constants.HardwareMap.TOP_SHOOTER_CAN_ID,
                CANSparkLowLevel.MotorType.kBrushless
        );


        bottomShooter = new LazyCANSparkFlex(
                Constants.HardwareMap.BOTTOM_SHOOTER_CAN_ID,
                CANSparkLowLevel.MotorType.kBrushless
        );

        //Restore factory defaults and initialize the vortex motors
        initVortax();


        motorTemperatureCount = new CountingDelay();

        shootSensor = new DigitalInput(Constants.HardwareMap.SHOOT_SENSOR_ID);

    }

    @Override
    public void onStart(double timestamp) {

        if (DriverStation.isAutonomous()){
            subsystemManager.setControlState(ShamperManager.controlState.DISABLED);
        } else {
            subsystemManager.setControlState(ShamperManager.controlState.DISABLED);
        }


    }

    @Override
    public void readPeriodicInputs() {
        subsystemManager.setShootSensor(getShootSensorValue());
    }

    @Override
    public void onLoop(double timestamp) {
        if (motorTemperatureCount.delay(10)) {
            SmartDashboard.putNumber("MotorHeat", topShooter.getMotorTemperature());

            if(topShooter.getMotorTemperature() > Constants.Shamper.OVERHEAT_THRESOLD) {
                subsystemManager.setMotorOverheat(true);
            } else {
                subsystemManager.setMotorOverheat(false);
            }

            motorTemperatureCount.reset();
        }


        targetSpeeds = new ShamperSpeeds(0,0,0);

        switch (subsystemManager.getControlState()) {

            case DISABLED -> {
                return;
            }
            case AUTOMATIC ->
                targetSpeeds = getSubsystemManager().getAutomaticVelocityTarget();
            case MANUAL ->
                targetSpeeds = subsystemManager.getVelocityState().getValue();
        }

        setShamperTargetSpeeds(targetSpeeds);

        double shamperMargin = subsystemManager.getShamperMargin();
        //Increase the margin if the shamper is in amp mode
        if (subsystemManager.getControlState() == ShamperManager.controlState.MANUAL
        && subsystemManager.getVelocityState() == ShamperManager.velocityState.AMP){
            shamperMargin = Constants.Shamper.SHAMPER_SETPOINT_MARGIN_AMP;
        } else if(subsystemManager.getControlState() == ShamperManager.controlState.MANUAL &&
        subsystemManager.getVelocityState() == ShamperManager.velocityState.IDLE){
            shamperMargin = SHAMPER_IDLE_MARGIN;
        }
        else if (subsystemManager.getControlState() == ShamperManager.controlState.MANUAL) {
            shamperMargin = SHAMPER_SETPOINT_MARGIN;
        }

        if (isOnTarget(targetSpeeds,shamperMargin)){
            subsystemManager.setShootingState(ShamperManager.shootingState.ON_TARGET);
        } else {
            subsystemManager.setShootingState(ShamperManager.shootingState.PREPARING);
        }
    }


    @Override
    public void writePeriodicOutputs() {
        SmartDashboard.putNumber("Outtake/Shamper/error/kicker", Math.abs(targetSpeeds.kickerShooterRpm() - kickerEncoder.getVelocity()));
//        SmartDashboard.putNumber("Outtake/Shamper/error/top", Math.abs(targetSpeeds.topShooterRpm() - topEncoder.getVelocity()));
        SmartDashboard.putNumber("Outtake/Shamper/error/top", Math.abs(targetSpeeds.topShooterRpm() - getTopEncoderVelocity()));
//        SmartDashboard.putNumber("Outtake/Shamper/error/bottom", Math.abs(targetSpeeds.bottomShooterRpm() - bottomEncoder.getVelocity()));
        SmartDashboard.putNumber("Outtake/Shamper/error/bottom", Math.abs(targetSpeeds.bottomShooterRpm() - getBottomEncoderVelocity()));
        SmartDashboard.putNumber("Outtake/Shamper/error/Top-Bottom",getBottomEncoderVelocity()-getTopEncoderVelocity());

        SmartDashboard.putNumber("Outtake/Shamper/setpoint/kicker", targetSpeeds.kickerShooterRpm());
        SmartDashboard.putNumber("Outtake/Shamper/setpoint/top", targetSpeeds.topShooterRpm());
        SmartDashboard.putNumber("Outtake/Shamper/setpoint/bottom", targetSpeeds.bottomShooterRpm());

        SmartDashboard.putNumber("Outtake/Shamper/arb ff/kicker", targetSpeeds.kickerShooterRpm() * Constants.Shamper.KICKER_FF);
        SmartDashboard.putNumber("Outtake/Shamper/arb ff/top", targetSpeeds.topShooterRpm() * Constants.Shamper.TOP_FF);
        SmartDashboard.putNumber("Outtake/Shamper/arb ff/bottom", targetSpeeds.bottomShooterRpm() * Constants.Shamper.BOTTOM_FF);

        SmartDashboard.putNumber("Outtake/Shamper/applied output/kicker", kickerMotor.getAppliedOutput());
        SmartDashboard.putNumber("Outtake/Shamper/applied output/top", topShooter.getAppliedOutput());
        SmartDashboard.putNumber("Outtake/Shamper/applied output/bottom", bottomShooter.getAppliedOutput());

        SmartDashboard.putNumber("Current/kicker shooter", kickerMotor.getOutputCurrent());
        SmartDashboard.putNumber("Current/top shooter", topShooter.getOutputCurrent());
        SmartDashboard.putNumber("Current/bottom shooter", bottomShooter.getOutputCurrent());
    }
    @Override
    public void onStop(double timestamp) {
        terminate();
    }

    @Override
    public void zeroSensors() {

    }


    @Override
    public void terminate() {

    }

    @Override
    public void outputData() {
        SmartDashboard.putString("Outtake/Shamper/controlState", subsystemManager.getControlState().toString());
        SmartDashboard.putString("Outtake/Shamper/velocityState", subsystemManager.getVelocityState().toString());
        SmartDashboard.putString("Outtake/Shamper/shootingState", subsystemManager.getShootingState().toString());
        SmartDashboard.putNumber("Outtake/Shamper/velocity/kicker", kickerEncoder.getVelocity());
//        SmartDashboard.putNumber("Outtake/Shamper/velocity/top", topEncoder.getVelocity());
        SmartDashboard.putNumber("Outtake/Shamper/velocity/top", getTopEncoderVelocity());
        SmartDashboard.putNumber("Outtake/Shamper/velocity/bottom", bottomEncoder.getVelocity());
        SmartDashboard.putNumber("Outtake/Shamper/velocity/bottom", getBottomEncoderVelocity());

        SmartDashboard.putNumber("Outtake/Shamper/iaccum/kicker", kickerPID.getIAccum());
        SmartDashboard.putNumber("Outtake/Shamper/iaccum/top", topPID.getIAccum());
        SmartDashboard.putNumber("Outtake/Shamper/iaccum/bottom", bottomPID.getIAccum());

        SmartDashboard.putBoolean("Outtake/Shamper/shootsensor", getShootSensorValue());
    }

    public boolean compare(double realspeed, double target, double margin){
        return (Math.abs(target - realspeed) <= margin);
    }

    public boolean isOnTarget (ShamperSpeeds target, double margin) {

        boolean compareKicker = compare(kickerEncoder.getVelocity(), target.kickerShooterRpm(), margin);
//        boolean compareTop = compare(topEncoder.getVelocity(), target.topShooterRpm(), margin);
        boolean compareTop = compare(getTopEncoderVelocity(), target.topShooterRpm(), margin);
//        boolean compareBottom = compare(bottomEncoder.getVelocity(), target.bottomShooterRpm(), margin);
        boolean compareBottom = compare(getBottomEncoderVelocity(), target.bottomShooterRpm(), margin);

        SmartDashboard.putBoolean("Outtake/Shamper/on target bools/compareKicker", compareKicker);
        SmartDashboard.putBoolean("Outtake/Shamper/on target bools/compareTop", compareTop);
        SmartDashboard.putBoolean("Outtake/Shamper/on target bools/compareBottom", compareBottom);




        return compareKicker && compareTop && compareBottom;
    }

    public void setShamperTargetSpeeds(ShamperSpeeds target){
        if (target.topShooterRpm() + target.bottomShooterRpm() + target.kickerShooterRpm() == 0){
            targetZero();
            return;
        }

        kickerPID.setReference(target.kickerShooterRpm(), CANSparkBase.ControlType.kVelocity, 0,
                target.kickerShooterRpm() * Constants.Shamper.KICKER_FF,
                SparkPIDController.ArbFFUnits.kVoltage);

        topPID.setReference(target.topShooterRpm()/CONVERSIONFACTOR_TOP_SHOOTER, CANSparkBase.ControlType.kVelocity, 0,
                target.topShooterRpm() * Constants.Shamper.TOP_FF,
                SparkPIDController.ArbFFUnits.kVoltage);

        bottomPID.setReference(target.bottomShooterRpm()/CONVERSIONFACTOR_BOTTOM_SHOOTER, CANSparkBase.ControlType.kVelocity, 0,
                target.bottomShooterRpm() * Constants.Shamper.BOTTOM_FF,
                SparkPIDController.ArbFFUnits.kVoltage);


    }
    public void targetZero(){
        kickerPID.setReference(0, CANSparkBase.ControlType.kVoltage,1);
        topPID.setReference(0, CANSparkBase.ControlType.kVoltage, 1);
        bottomPID.setReference(0, CANSparkBase.ControlType.kVoltage, 1);
    }

    private boolean getShootSensorValue(){
        return !shootSensor.get();
    }

    public void initVortax(){
        DataLogManager.log("Warning: resetting spark vortex");
        topShooter.restoreFactoryDefaults();
        topShooter.setSmartCurrentLimit(CURRENT_LIMIT_TOP);
        topShooter.setInverted(true);
        topShooter.setIdleMode(CANSparkBase.IdleMode.kCoast);
        topEncoder = topShooter.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 7168);
        topEncoder.setVelocityConversionFactor(1.0);
//        topEncoder.setVelocityConversionFactor(Constants.Shamper.CONVERSIONFACTOR_TOP_SHOOTER);
        topPID = topShooter.getPIDController();
        topPID.setP(Constants.Shamper.TOP_KP,0);
        topPID.setI(Constants.Shamper.TOP_KI,0);
        topPID.setIZone(Constants.Shamper.SHAMPER_KI_DEADZONE, 0);
        topPID.setIMaxAccum(Constants.Shamper.SHAMPER_KI_MAX_ACCUM, 0);
        topShooter.burnFlash();

        bottomShooter.restoreFactoryDefaults();
        bottomShooter.setSmartCurrentLimit(CURRENT_LIMIT_BOTTOM);
        bottomShooter.setInverted(true);
        bottomShooter.setIdleMode(CANSparkBase.IdleMode.kCoast);
        bottomEncoder = bottomShooter.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 7168);
//        bottomEncoder.setVelocityConversionFactor(Constants.Shamper.CONVERSIONFACTOR_BOTTOM_SHOOTER);
        bottomEncoder.setVelocityConversionFactor(1.0);
        bottomPID = bottomShooter.getPIDController();
        bottomPID.setP(Constants.Shamper.BOTTOM_KP,0);
        bottomPID.setI(Constants.Shamper.BOTTOM_KI,0);
        bottomPID.setIZone(Constants.Shamper.SHAMPER_KI_DEADZONE, 0);
        bottomPID.setIMaxAccum(Constants.Shamper.SHAMPER_KI_MAX_ACCUM, 0);
        bottomShooter.burnFlash();

    }

    private double getTopEncoderVelocity() {
        return topEncoder.getVelocity() * CONVERSIONFACTOR_TOP_SHOOTER;
    }
    private double getBottomEncoderVelocity() {
        return bottomEncoder.getVelocity() * CONVERSIONFACTOR_BOTTOM_SHOOTER;
    }

}