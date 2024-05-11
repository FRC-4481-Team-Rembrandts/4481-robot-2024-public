package frc.team4481.lib.swerve;

import com.revrobotics.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.team4481.lib.hardware.LazyCANSparkFlex;

public class SwerveDriveHelper 
{
//    public LazyCANSparkWeekend driveMotor;
    public LazyCANSparkFlex driveMotor;
    public SimpleMotorFeedforward FFController;

    /**
     *
     * @param motorID
     * @param isInverted
     * @param PIDValues
     * @param FFValues
     */
    public SwerveDriveHelper(
            int motorID,
            boolean isInverted,
            double encoderPositionConversionFactor,
            double encoderVelocityConversionFactor,
            PIDValueContainer PIDValues,
            FFValueContainer FFValues,
            int current_limit,
            CANSparkBase.IdleMode idleMode,
            boolean isNeoVortex
            )
    {
        if (isNeoVortex) {
            driveMotor = new LazyCANSparkFlex(motorID, CANSparkBase.MotorType.kBrushless);
//            driveMotor = new LazyCANSparkWeekend(motorID, CANSparkBase.MotorType.kBrushless);
        } else {
            driveMotor = new LazyCANSparkFlex(motorID, CANSparkBase.MotorType.kBrushless);
//            driveMotor = new LazyCANSparkWeekend(motorID, CANSparkBase.MotorType.kBrushless);
        }

        driveMotor.restoreFactoryDefaults();
        driveMotor.setIdleMode(idleMode);
        driveMotor.setInverted(isInverted);
        driveMotor.setSmartCurrentLimit(current_limit);

        RelativeEncoder relativeEncoder = driveMotor.getEncoder();
//        relativeEncoder.setPositionConversionFactor(encoderPositionConversionFactor);
//        relativeEncoder.setVelocityConversionFactor(encoderVelocityConversionFactor);

        SparkPIDController controller = driveMotor.getPIDController();
        controller.setFeedbackDevice(relativeEncoder);
        controller.setP(PIDValues.kP);
        controller.setI(PIDValues.kI);
        controller.setD(PIDValues.kD);
        driveMotor.burnFlash();

        FFController = new SimpleMotorFeedforward(
                FFValues.kS,
                FFValues.kV,
                FFValues.kA
        );
    }
}
