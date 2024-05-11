package frc.team4481.lib.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.*;

import edu.wpi.first.math.controller.ArmFeedforward;


public class SwerveTurnHelper
{
    public CANSparkMax turnMotor;
    public ArmFeedforward FFController;
    public CANcoder turnCanCoder;
    public boolean isSDS = false;

    /**
     * Constructor for SDS swerve module
     * @param motorID
     * @param PIDValues
     * @param FFValues
     * @param current_limit
     * @param canCoderID
     * @param canCoderOffsetDegrees
     * @param turnGearRatio
     */
    public SwerveTurnHelper(
            int motorID,
            PIDValueContainer PIDValues,
            FFValueContainer FFValues,
            int current_limit,
            int canCoderID,
            double canCoderOffsetDegrees,
            double turnGearRatio
    ) {
        this.isSDS = true;

        turnMotor = new CANSparkMax(motorID, CANSparkMax.MotorType.kBrushless);
        turnMotor.restoreFactoryDefaults();
        turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        turnMotor.setSmartCurrentLimit(current_limit);

        CANcoderConfiguration config = new CANcoderConfiguration();
        
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        config.MagnetSensor.MagnetOffset = canCoderOffsetDegrees;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        turnCanCoder = new CANcoder(canCoderID);
        turnCanCoder.getConfigurator().apply(config, 250);
        // turnCanCoder.setStatusFramePeriod(CANcoderStatusFrame.SensorData, 100, 250);
        

        RelativeEncoder relativeEncoder = turnMotor.getEncoder();
        relativeEncoder.setPositionConversionFactor(2.0 * Math.PI * turnGearRatio); // Using radians
        relativeEncoder.setVelocityConversionFactor(2.0 * Math.PI * turnGearRatio / 60);
        relativeEncoder.setPosition(Math.toRadians(turnCanCoder.getAbsolutePosition().getValue()*360));

        SparkPIDController controller = turnMotor.getPIDController();
        controller.setP(PIDValues.kP);
        controller.setI(PIDValues.kI);
        controller.setD(PIDValues.kD);
        controller.setFeedbackDevice(relativeEncoder);
        turnMotor.burnFlash();

        FFController = new ArmFeedforward(
                FFValues.kS,
                0.0,
                FFValues.kV,
                FFValues.kA
        );
    }

    /**
     * Constructor for REV swerve module
     * @param motorID
     * @param PIDValues
     * @param FFValues
     * @param current_limit
     */
    public SwerveTurnHelper(
            int motorID,
            PIDValueContainer PIDValues,
            FFValueContainer FFValues,
            int current_limit
    ) {
        turnMotor = new CANSparkMax(motorID, CANSparkMax.MotorType.kBrushless);
        turnMotor.restoreFactoryDefaults();
        turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        turnMotor.setSmartCurrentLimit(current_limit);
        turnMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 10);

        SparkAbsoluteEncoder absoluteEncoder = turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        absoluteEncoder.setInverted(true);

        // Convert from rotations to radians
        // And from RMP to rad/s
        absoluteEncoder.setPositionConversionFactor(2 * Math.PI);
        absoluteEncoder.setVelocityConversionFactor(2 * Math.PI / 60);

        SparkPIDController controller = turnMotor.getPIDController();
        controller.setFeedbackDevice(absoluteEncoder);

        controller.setPositionPIDWrappingEnabled(true);
        controller.setPositionPIDWrappingMinInput(0);
        controller.setPositionPIDWrappingMaxInput(2 * Math.PI);
        controller.setP(PIDValues.kP);
        controller.setI(PIDValues.kI);
        controller.setD(PIDValues.kD);
        turnMotor.burnFlash();

        FFController = new ArmFeedforward(
                FFValues.kS,
                0.0,
                FFValues.kV,
                FFValues.kA
        );
    }

}
