package frc.team4481.robot.subsystems;


import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.lib.hardware.LazyCANSparkMax;
import frc.team4481.lib.subsystems.SubsystemBase;
import frc.team4481.lib.util.CountingDelay;
import frc.team4481.robot.Constants;

public class Climber extends SubsystemBase<ClimberManager> {
    private LazyCANSparkMax climberMotorLeft;
    private LazyCANSparkMax climberMotorRight;
    private boolean isLeftMotorCalibrated;
    private boolean isRightMotorCalibrated;
    private RelativeEncoder encoderLeft;
    private RelativeEncoder encoderRight;
    private CountingDelay countingDelay;
    private CountingDelay moveUpDelay;



    public Climber(){
        name = "Climber";
        subsystemManager = new ClimberManager();

        climberMotorLeft = new LazyCANSparkMax(Constants.HardwareMap.CLIMBER_LEFT_ID, CANSparkLowLevel.MotorType.kBrushless);
        encoderLeft = climberMotorLeft.getEncoder();
        climberMotorLeft.setInverted(true);
        climberMotorLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);
        climberMotorLeft.setSmartCurrentLimit(Constants.Climber.CURRENT_LIMIT);
        climberMotorLeft.burnFlash();


        climberMotorRight = new LazyCANSparkMax(Constants.HardwareMap.CLIMBER_RIGHT_ID, CANSparkLowLevel.MotorType.kBrushless);
        encoderRight = climberMotorRight.getEncoder();
        climberMotorRight.setInverted(false);
        climberMotorRight.setIdleMode(CANSparkBase.IdleMode.kBrake);
        climberMotorRight.setSmartCurrentLimit(Constants.Climber.CURRENT_LIMIT);
        climberMotorRight.burnFlash();

        isLeftMotorCalibrated = false;
        isRightMotorCalibrated = false;

    }

    @Override
    public void onStart(double timestamp) {
        isLeftMotorCalibrated = false;
        isRightMotorCalibrated = false;
        if (DriverStation.isAutonomous()){
            subsystemManager.setControlState(ClimberManager.controlState.DISABLED);
        } else {
            subsystemManager.setControlState(ClimberManager.controlState.CALIBRATING);
        }
        countingDelay = new CountingDelay();
        countingDelay.reset();

        moveUpDelay = new CountingDelay();
        moveUpDelay.reset();

    }

    @Override
    public void readPeriodicInputs() {    }

    @Override
    public void onLoop(double timestamp) {

        switch (subsystemManager.getControlState()) {
            case DISABLED:
                    climberMotorRight.set(0);
                    climberMotorLeft.set(0);
                break;

            case CALIBRATING:
                // calibrating left motor
                if (countingDelay.delay(0.5) && climberMotorLeft.getOutputCurrent() >= Constants.Climber.CALIBRATION_CHECK_LIMMIT){
                    isLeftMotorCalibrated = true;
                    climberMotorLeft.set(0);
                    encoderLeft.setPosition(0);
                } else if (isLeftMotorCalibrated) {
                    climberMotorLeft.set(0);
                } else {
                    climberMotorLeft.set(Constants.Climber.CALIBRATION_SPEED);
                }

                // calibrating right motor
                if (countingDelay.delay(0.5) && climberMotorRight.getOutputCurrent() >= Constants.Climber.CALIBRATION_CHECK_LIMMIT) {
                    isRightMotorCalibrated = true;
                    climberMotorRight.set(0);
                    encoderRight.setPosition(0);
                } else if (isRightMotorCalibrated){
                    climberMotorRight.set(0);
                } else {
                    climberMotorRight.set(Constants.Climber.CALIBRATION_SPEED);
                }

                // sets control state
                if (isLeftMotorCalibrated && isRightMotorCalibrated){
                     if(moveUpDelay.delay(0.4)){
                         subsystemManager.setControlState(ClimberManager.controlState.MANUAL);
                     } else {
                         climberMotorLeft.set(-Constants.Climber.CALIBRATION_SPEED);
                         climberMotorRight.set(-Constants.Climber.CALIBRATION_SPEED);
                     }
                } else {
                    moveUpDelay.reset();
                    subsystemManager.setControlState(ClimberManager.controlState.CALIBRATING);

                }
//;
                break;
            case MANUAL:
                setClimberSpeed(climberMotorRight, encoderRight);
                setClimberSpeed(climberMotorLeft, encoderLeft);

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
        SmartDashboard.putString("Climber/controlState", subsystemManager.getControlState().toString());
        SmartDashboard.putNumber("Climber/desiredSpeed", subsystemManager.getClimbingSpeed());
        SmartDashboard.putBoolean("Climber/calibratedLeft", isLeftMotorCalibrated);
        SmartDashboard.putBoolean("Climber/calibratedRight", isRightMotorCalibrated);

        SmartDashboard.putNumber("Current/climber left", climberMotorLeft.getOutputCurrent());
        SmartDashboard.putNumber("Current/climber right", climberMotorRight.getOutputCurrent());

        SmartDashboard.putNumber("Climber/left pos", encoderLeft.getPosition());
        SmartDashboard.putNumber("Climber/right pos", encoderRight.getPosition());
    }

    public void setClimberSpeed(LazyCANSparkMax motor, RelativeEncoder encoder){
        if(encoder.getPosition() >= Constants.Climber.HIGH_SETPOINT - Constants.Climber.MARGE &&
                subsystemManager.getClimbingSpeed() < 0) {
            motor.set(0);
        } else if (encoder.getPosition() <= Constants.Climber.LOW_SETPOINT + Constants.Climber.MARGE &&
                subsystemManager.getClimbingSpeed() > 0){
            motor.set(0);
        } else {
            // gets value from leftstick Y
            motor.set(-subsystemManager.getClimbingSpeed());
        }
    }

}