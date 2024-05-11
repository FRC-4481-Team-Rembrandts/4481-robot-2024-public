package frc.team4481.robot.subsystems;


import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.team4481.lib.hardware.LazyCANSparkMax;
import frc.team4481.lib.hardware.LazyCANSparkFlex;
import frc.team4481.lib.subsystems.SubsystemBase;
import frc.team4481.lib.subsystems.SubsystemHandler;
import frc.team4481.lib.util.CountingDelay;
import static frc.team4481.robot.Constants.Intake.*;
import static java.lang.Math.abs;

import frc.team4481.robot.Constants;
import frc.team4481.robot.configuration.ConfigurationHandler;


public class Intake extends SubsystemBase<IntakeManager> {
    private final SubsystemHandler subsystemHandler = SubsystemHandler.getInstance();
    private final ConfigurationHandler configHandler = ConfigurationHandler.getInstance();

    //motors
    private LazyCANSparkFlex intakeMotor;
    private LazyCANSparkFlex fullWidthRoller;
    private LazyCANSparkMax storageMotor;

    //sensoren
    private DigitalInput lowSensor;
    private DigitalInput highSensor;

    private CountingDelay counterHighSensor;

    private LazyCANSparkFlex feederShooter;
    private RelativeEncoder feederEncoder;
    private SparkPIDController feederPID;
    private CountingDelay rampUpDelay;

    private boolean intakeStatedUp = false;
    private double intakeRampUpSpeed = 0.0;
    private double storageRampUpSpeed = 0.0;

    private Mechanism2d intakeMechanism;
    private MechanismLigament2d intakeMechanismIntake;
    private MechanismLigament2d intakeMechanismStorage;

    private CountingDelay lowSensorFilterDelay = new CountingDelay();
    private CountingDelay highSensorFilterDelay = new CountingDelay();

    public Intake() {
        name = "Intake";
        subsystemManager = new IntakeManager();

        counterHighSensor = new CountingDelay();

        //add motors
        intakeMotor = new LazyCANSparkFlex(Constants.HardwareMap.IN_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        intakeMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        intakeMotor.setSmartCurrentLimit(CURRENT_LIMIT_INTAKE);
        intakeMotor.setInverted(false);

        fullWidthRoller = new LazyCANSparkFlex(Constants.HardwareMap.FULL_WIDTH_ROLLER_ID, CANSparkLowLevel.MotorType.kBrushless);
        fullWidthRoller.setSmartCurrentLimit(CURRENT_LIMIT_INTAKE);
        fullWidthRoller.setInverted(false);

        storageMotor = new LazyCANSparkMax(Constants.HardwareMap.STORAGE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        storageMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        storageMotor.setSmartCurrentLimit(CURRENT_LIMIT_STORAGE);
        storageMotor.setInverted(true);

        feederShooter = new LazyCANSparkFlex(Constants.HardwareMap.FEEDER_SHOOTER_CAN_ID, CANSparkLowLevel.MotorType.kBrushless);
        feederShooter.setIdleMode(CANSparkBase.IdleMode.kCoast);
        feederShooter.setInverted(false);
        feederEncoder = feederShooter.getEncoder();
        feederEncoder.setVelocityConversionFactor(Constants.Intake.CONVERSIONFACTOR_FEEDER);
        feederPID = feederShooter.getPIDController();
        feederPID.setP(Constants.Intake.FEEDER_KP, 0);
        feederShooter.burnFlash();

        //Sensors
        lowSensor = new DigitalInput(Constants.HardwareMap.LOW_SENSOR_ID);
        highSensor = new DigitalInput(Constants.HardwareMap.HIGH_SENSOR_ID);

        // Mechanism2d
        intakeMechanism = new Mechanism2d(3, 3);
        MechanismRoot2d intakeMechanismRoot = intakeMechanism.getRoot("Storage", 1.311, 0);
        intakeMechanismIntake = intakeMechanismRoot.append(
                new MechanismLigament2d("Intake", 0.20, 90, 30, new Color8Bit(Color.kGray)));
        intakeMechanismStorage = intakeMechanismIntake.append(
                new MechanismLigament2d("Storage", 0.26, 0, 15, new Color8Bit(Color.kWhite)));
    }

    @Override
    public void onStart(double timestamp) {
        if (DriverStation.isAutonomous()) {
            subsystemManager.setControlState(IntakeManager.controlState.HOLD);
        } else {
            subsystemManager.setControlState(IntakeManager.controlState.DISABLED);
        }
    }

    @Override
    public void readPeriodicInputs() {
        subsystemManager.setStorageOccupied(getHighSensorValue() || getLowerSensorValue());
        subsystemManager.setSensorHigh(getHighSensorValue());
        subsystemManager.setSensorLow(getLowerSensorValue());

        intakeMechanismIntake.setColor(booleanToColor(getLowerSensorValue()));
        intakeMechanismStorage.setColor(booleanToColor(getHighSensorValue()));
    }

    @Override
    public void onLoop(double timestamp) {

        switch (subsystemManager.getControlState()) {
            case DISABLED:
                //do nothing
                intakeRampUp(0.0, INTAKE_RAMP_UP_TIME);
                storeRamUp(0.0, STORAGE_RAMP_UP_TIME);
                feederCoast();

                counterHighSensor.reset();

                break;
            case INTAKE:
                //intake is used for pikking the nodes of the floor
                if (getLowerSensorValue() || getHighSensorValue()) {
                    subsystemManager.setControlState(IntakeManager.controlState.STORE);
                } else {
                    intakeRampUp(INTAKE_SPEED, INTAKE_RAMP_UP_TIME);
                    storeRamUp(STORE_SPEED,INTAKE_SPEED_STORAGE);
                    feedControl(FEEDER_SLOW_SPEED);
                }


                break;
            case REVERSE_INTAKE:
                //reverse intake is for taking the node uit of the robot
                intakeRampUp(-INTAKE_SPEED, INTAKE_RAMP_UP_TIME);
                storeControl(-INTAKE_SPEED);
                feederCoast();
                break;

            case INTAKE_FEEDER:
                //intake feeder is for feeding the shamper a note
                intakeRampUp(FEEDING_SPEED_INTAKE, INTAKE_RAMP_UP_TIME);
                storeControl(FEEDING_SPEED_STORAGE);
                feedControl(FEEDING_RPM);
                break;

            case TURBO_FEEDER:
                intakeControl(TURBO_FEEDER_SPEED);
                storeControl(FEEDING_SPEED_STORAGE);
                feedControl(FEEDING_RPM);
                break;

            case STORE:
                //store is for getting the node up until the hardstop

                if (getHighSensorValue()) {
                    if (counterHighSensor.delay(HIGH_SENSOR_DELAY)) {
                        subsystemManager.setControlState(IntakeManager.controlState.HOLD);
                    } else {
                        intakeRampUp(STORE_SPEED, INTAKE_RAMP_UP_TIME);
                        storeRamUp(STORE_SPEED_STORAGE, STORAGE_RAMP_UP_TIME);
                        feedControl(FEEDER_SLOW_SPEED);
                    }
                } else {
                    counterHighSensor.reset();

                    intakeRampUp(STORE_SPEED, INTAKE_RAMP_UP_TIME);
                    storeRamUp(STORE_SPEED_STORAGE, STORAGE_RAMP_UP_TIME);
                    feedControl(FEEDER_SLOW_SPEED);
                }

                break;
            case HOLD:
                //hold is keeping the node in the storage
                intakeRampUp(0.0, INTAKE_RAMP_UP_TIME);
                storeRamUp(0.0, STORAGE_RAMP_UP_TIME);
                feedControl(FEEDER_SLOW_SPEED);
                counterHighSensor.reset();
                if (!getLowerSensorValue() && !getHighSensorValue()) {
                    subsystemManager.setControlState(IntakeManager.controlState.DISABLED);
                }

                break;
            case SOURCE_INTAKE:
                //source intake to pickup from source
                if (getLowerSensorValue()) {
                    subsystemManager.setControlState(IntakeManager.controlState.STORE);
                } else {
                    storeControl(INTAKE_SOURCE_STORAGE_SPEED);
                    feedControl(-200);
                }

                break;
        }
    }

    @Override
    public void writePeriodicOutputs() {
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

        //getControltate
        SmartDashboard.putString("Intake/IntakeState", subsystemManager.getControlState().toString());

        //motors feedback
        SmartDashboard.putBoolean("Intake/sensor low", getLowerSensorValue());
        SmartDashboard.putBoolean("Intake/sensor high", getHighSensorValue());
        SmartDashboard.putBoolean("Intake/sensor low raw", !lowSensor.get());
        SmartDashboard.putBoolean("Intake/sensor high raw", !highSensor.get());
        SmartDashboard.putBoolean("Intake/storage occupied", subsystemManager.isStorageOccupied());

        SmartDashboard.putNumber("Intake/feeder speed", feederEncoder.getVelocity());
        SmartDashboard.putNumber("Intake/feeder applied output", feederShooter.getAppliedOutput());

        SmartDashboard.putNumber("Current/fullwidth", fullWidthRoller.getOutputCurrent());
        SmartDashboard.putNumber("Current/intake", intakeMotor.getOutputCurrent());
        SmartDashboard.putNumber("Current/storage", storageMotor.getOutputCurrent());
        SmartDashboard.putNumber("Current/feeder", feederShooter.getOutputCurrent());

        SmartDashboard.putNumber("Intake/intake speed", intakeMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Intake/intake fullwidth roller speed", fullWidthRoller.getEncoder().getVelocity());
        SmartDashboard.putNumber("Intake/storage speed", storageMotor.getEncoder().getVelocity());

        SmartDashboard.putData("Intake/mechanism", intakeMechanism);
    }

    public void intakeControl(double intakeSpeed) {
        intakeMotor.set(intakeSpeed);
        fullWidthRoller.set(intakeSpeed);
    }

    public void storeControl(double storageSpeed) {
        storageMotor.set(storageSpeed);
    }

    public void feedControl(double feedingSpeed) {
        SmartDashboard.putNumber("Intake/feeder setpoint", feedingSpeed);
        SmartDashboard.putNumber("Intake/feeder arb ff", feedingSpeed * Constants.Intake.FEEDER_FF);
        feederPID.setReference(feedingSpeed, CANSparkBase.ControlType.kVelocity, 0,
                feedingSpeed * Constants.Intake.FEEDER_FF,
                SparkPIDController.ArbFFUnits.kVoltage);
    }

    private boolean getLowerSensorValue() {
        //When the sensor returns false, something is in front of it
        boolean triggered = !lowSensor.get();

        //Reset the time when the sensor is not triggered
        //This makes sure that the sensor should be on for a certain time to return true
        if (!triggered){
            lowSensorFilterDelay.reset();
        }

        return lowSensorFilterDelay.delay(SENSOR_FILTER_TIME);
    }

    private boolean getHighSensorValue() {
        //When the sensor returns false, something is in front of it
        boolean triggered = !highSensor.get();

        //Reset the time when the sensor is not triggered
        //This makes sure that the sensor should be on for a certain time to return true
        if (!triggered){
            highSensorFilterDelay.reset();
        }

        return highSensorFilterDelay.delay(SENSOR_FILTER_TIME);
    }

    public void feederCoast() {
        feederPID.setReference(0, CANSparkBase.ControlType.kDutyCycle, 1);
    }

    public void intakeRampUp(double intakeSpeed, double timeToTarget) {
        if ((abs(intakeRampUpSpeed) < abs(intakeSpeed))) {
            double increment = intakeSpeed * Constants.kLooperDt / timeToTarget;
            intakeRampUpSpeed += increment;
            intakeControl(intakeRampUpSpeed);
        }else if (abs(intakeRampUpSpeed) >= abs(intakeSpeed)){
            if (intakeSpeed == 0.0) {
                intakeRampUpSpeed = 0.0;
            }
            intakeControl(intakeSpeed);
        }
    }

    public void storeRamUp(double storageSpeed, double timeToTarget){
        if ((abs(storageRampUpSpeed) < abs(storageSpeed))){
            double increment = storageSpeed * Constants.kLooperDt / timeToTarget;
            storageRampUpSpeed += increment;
            storeControl(storageRampUpSpeed);
        }else if (abs(storageRampUpSpeed) >= abs(storageSpeed)){
            if (storageSpeed == 0.0) {
                storageRampUpSpeed = 0.0;
            }
            storeControl(storageSpeed);
        }
    }

    private Color8Bit booleanToColor(boolean bool) {
        return bool ? new Color8Bit(Color.kRed) : new Color8Bit(Color.kWhite);
    }
}