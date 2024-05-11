package frc.team4481.lib.hardware;

import com.revrobotics.*;
import com.revrobotics.jni.CANSparkMaxJNI;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Sends only new commands to the motor to reduce CAN usage.
 *
 * Copyright 2019 FRC Team 3476 Code Orange
 *
 * Edited by: FRC 4481 Team Rembrandts 2022
 * Edit 2024: added extra checker to check if parameters are set correctly
 */
public class LazyCANSparkFlex extends CANSparkFlex {
    private final static double EPSILON = 1.0e-6;
    private double prevValue = 0;
    private double prevVoltage = 100;
    private final int VORTEX_ENCODER_TICKS_PER_ROTATION = 7168;
    private final int MAX_ATTEMPTS = 20;
    private final double APPLY_PARAMETER_WAIT_TIME = 0.1;
    private final double BURN_TO_FLASH_WAIT_TIME = 0.1;
    private final int deviceId;



    public LazyCANSparkFlex(int deviceId, MotorType type) {
        super(deviceId, type);
        super.restoreFactoryDefaults();
        this.deviceId = deviceId;
    }


    @Override
    public RelativeEncoder getEncoder() {
        return getEncoder(SparkRelativeEncoder.Type.kQuadrature, VORTEX_ENCODER_TICKS_PER_ROTATION);
    }


    /**
     * Function to set a parameter in the vortex and make sure that this parameter is 100% set correctly
     *
     * @param parameterSetter The function to use to set the desired parameter
     * @param parameterCheckSupplier Boolean comparison with which to check if the parameter is set correctly
     * @param errorMessage The text to display if an error occurs
     * @return REVLibError message
     */
    private REVLibError applyParameter(Supplier<REVLibError> parameterSetter, BooleanSupplier parameterCheckSupplier, String errorMessage) {
        //If the parameter is already correct, we are finished immediately and can celebrate more weekend!
        if (parameterCheckSupplier.getAsBoolean()) {
            return REVLibError.kOk;
        }

        REVLibError status = REVLibError.kError;
        for (int i = 0; i < MAX_ATTEMPTS; i++) {
            status = parameterSetter.get();
            //If the parameter we get is the correct one and we get no error, we good and we can continue
            if (parameterCheckSupplier.getAsBoolean() && status == REVLibError.kOk) break;
            Timer.delay(APPLY_PARAMETER_WAIT_TIME);
        }

        if (status != REVLibError.kOk){
            System.err.println(deviceId + errorMessage + "-" + status.toString());
        }

        return status;
    }

    @Override
    public REVLibError setSmartCurrentLimit(int limit) {
        System.out.println("Weekendly setting current limit for motor " + deviceId);

        throwIfClosed();
        return applyParameter(
                () -> super.setSmartCurrentLimit(limit),
                () -> CANSparkMaxJNI.c_SparkMax_GetSmartCurrentStallLimit(sparkMaxHandle) == limit,
                "setSmartCurrentLimit failed"
        );
    }

    @Override
    public void setInverted(boolean isInverted) {
        System.out.println("Weekendly setting inversion for motor " + deviceId);

        throwIfClosed();
        applyParameter(
                () -> REVLibError.fromInt(CANSparkMaxJNI.c_SparkMax_SetInverted(sparkMaxHandle, isInverted)),
                () -> super.getInverted() == isInverted,
            "setInverted failed"
        );
    }

    public REVLibError burnFlash(){
        REVLibError burnFlashResult;
        Timer.delay(BURN_TO_FLASH_WAIT_TIME);
        burnFlashResult = super.burnFlash();
        Timer.delay(BURN_TO_FLASH_WAIT_TIME);
        return burnFlashResult;
    }
}