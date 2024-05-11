package frc.team4481.lib.swerve;


import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.*;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4481.lib.hardware.LazyCANSparkFlex;

import static frc.team4481.robot.Constants.Drivetrain.DRIVE_POSITION_CONVERSION_FACTOR;
import static frc.team4481.robot.Constants.Drivetrain.DRIVE_VELOCITY_CONVERSION_FACTOR;

/*This class is modified for SDS*/

public class SwerveModule
{

    public boolean isSDS;
    // Driving
    private LazyCANSparkFlex driveMotor;
    private RelativeEncoder driveEncoder;
    private SparkPIDController drivePIDController;
    private SimpleMotorFeedforward driveFFController;

    // Turning
    private CANSparkMax turnMotor;
    private SparkPIDController turnPIDController;

    private ArmFeedforward turnFFController;

    private RelativeEncoder turnEncoder;
    private CANcoder turnAbsoluteEncoder;

    private double resetIteration = 0;
    private static final int ENCODER_RESET_ITERATIONS = 500;
    private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);

    private SwerveModulePhysicalConstants consts;

    public SwerveModule(
            int id,
            SwerveTurnHelper turnHelper,
            SwerveDriveHelper driveHelper
    )
    {
        isSDS = turnHelper.isSDS;
        SmartDashboard.putBoolean("DT/swerve module is SDS/" + id, isSDS);

        // Set turn stuff
        turnMotor = turnHelper.turnMotor;
        turnEncoder = turnMotor.getEncoder();
        turnPIDController = turnMotor.getPIDController();
        turnFFController = turnHelper.FFController;

        if (isSDS) {
            turnAbsoluteEncoder = turnHelper.turnCanCoder;
        }

        // Set drive stuff
        driveMotor = driveHelper.driveMotor;
        driveEncoder = driveMotor.getEncoder();
        drivePIDController = driveMotor.getPIDController();
        driveFFController = driveHelper.FFController;
    }

    /**
     * Gets the current {@code SwerveModuleState} of this module
     *
     * @return current state of this module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getAbsoluteAngle());
    }

    /**
     * Sets the desired velocity and angle of this swerve module
     *
     * @param desiredState the desired {@code SwerveModuleState} of this module
     */
    public void setDesiredState(SwerveModuleState desiredState)
    {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001)
        {
            stopModule();
            return;
        }
        // Optimize the desired state such that the wheel has to turn the smallest possible angle
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getAbsoluteAngle());

        // Turning
        double targetAngle = optimizedState.angle.getRadians();

        final double turnArbFFComponent = turnFFController.calculate(targetAngle, 0);

        double adjustedReferenceAngleRadians = targetAngle;
        if (isSDS) {

            double currentAngleRadians = getAbsoluteAngle().getRadians();

            double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
            if (currentAngleRadiansMod < 0.0) {
                currentAngleRadiansMod += 2.0 * Math.PI;
            }

            // The reference angle has the range [0, 2pi) but the Neo's encoder can go above that
            adjustedReferenceAngleRadians = targetAngle + currentAngleRadians - currentAngleRadiansMod;
            if (targetAngle - currentAngleRadiansMod > Math.PI) {
                adjustedReferenceAngleRadians -= 2.0 * Math.PI;
            } else if (targetAngle - currentAngleRadiansMod < -Math.PI) {
                adjustedReferenceAngleRadians += 2.0 * Math.PI;
            }
        }

        turnPIDController.setReference(
                adjustedReferenceAngleRadians,
                CANSparkBase.ControlType.kPosition,
                0,
                turnArbFFComponent,
                SparkPIDController.ArbFFUnits.kVoltage
        );
        SmartDashboard.putNumber("DT/" + driveMotor.getDeviceId() + "/Target angle", adjustedReferenceAngleRadians);
        SmartDashboard.putNumber("DT/" + driveMotor.getDeviceId() + "/Current angle", getAbsoluteAngle().getRadians());

        // Driving

        double targetVelocity = optimizedState.speedMetersPerSecond;

        double arbFFComponent = driveFFController.calculate(targetVelocity);
        drivePIDController.setReference(
                targetVelocity * DRIVE_VELOCITY_CONVERSION_FACTOR,
                CANSparkBase.ControlType.kVelocity,
                0,
                arbFFComponent,
                SparkPIDController.ArbFFUnits.kVoltage
        );
        SmartDashboard.putNumber("DT/" + driveMotor.getDeviceId() + "/Target velocity", targetVelocity);
        SmartDashboard.putNumber("DT/" + driveMotor.getDeviceId() + "/Current velocity", getDriveVelocity());
        SmartDashboard.putNumber("DT/" + driveMotor.getDeviceId() + "/Absolute velocity error", Math.abs(targetVelocity - getDriveVelocity()));
        SmartDashboard.putNumber("DT/" + driveMotor.getDeviceId() + "/Arb ff volts", arbFFComponent);
        SmartDashboard.putNumber("DT/" + driveMotor.getDeviceId() + "/Applied volts", driveMotor.getAppliedOutput() * 12.0);

        SmartDashboard.putNumber("Current/turn motor " + turnMotor.getDeviceId(), turnMotor.getOutputCurrent());
        SmartDashboard.putNumber("Current/drive motor " + driveMotor.getDeviceId(), driveMotor.getOutputCurrent());
    }

    /**
     * Gets the current absolute angle of the swerve module
     *
     * @return current {@code Rotation2d} of absolute turning angle
     */
    public Rotation2d getAbsoluteAngle()
    {
        double angle;
        if (isSDS) {
            angle = turnEncoder.getPosition();

            // Reset the NEO's encoder periodically when the module is not rotating.
            // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't fully set up, and we don't
            // end up getting a good reading. If we reset periodically this won't matter anymore.
            if (turnEncoder.getVelocity() < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
                if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
                    resetIteration = 0;

                    // The cancoder outputs 0 to 1
                    double absAngle = Math.toRadians(turnAbsoluteEncoder.getAbsolutePosition().getValue()*360);
                    absAngle %= 2.0 * Math.PI;
                    if (absAngle < 0.0) {
                        absAngle += 2.0 * Math.PI;
                    }
                    turnEncoder.setPosition(absAngle);
                    angle = absAngle;
                }
            } else {
                resetIteration = 0;
            }


        } else {
            SparkAbsoluteEncoder encoder = turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

            angle = encoder.getPosition();
        }

        return new Rotation2d(angle);
    }

    public double getRelativeAngle() {
//        double currentAngleRadians = turnEncoder.getPosition();
//        double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
//        if (currentAngleRadiansMod < 0.0) {
//            currentAngleRadiansMod += 2.0 * Math.PI;
//        }
        return turnEncoder.getPosition();
    }

    /**
     * Gets the current velocity of the swerve module in m/s.
     *
     * @return current velocity in m/s
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity() * DRIVE_VELOCITY_CONVERSION_FACTOR;
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveEncoder.getPosition() * DRIVE_POSITION_CONVERSION_FACTOR,
                getAbsoluteAngle()
        );
    }

    /**
     * Set both motors of o {@code SwerveModule} to 0 output.
     */
    private void stopModule()
    {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    // Methods to edit Swerve module parameters
    public void setInverted(boolean isInverted) {
        driveMotor.setInverted(isInverted);
    }

    public boolean getInverted() {
        return driveMotor.getInverted();
    }

    public void setIdleMode(CANSparkBase.IdleMode mode) {
        driveMotor.setIdleMode(mode);
        turnMotor.setIdleMode(mode);

    }

    public CANSparkBase.IdleMode getIdleMode() {
        return driveMotor.getIdleMode();
    }

    public void setClosedLoopRampRate(double rate) {
        driveMotor.setClosedLoopRampRate(rate);
    }

    public double getClosedLoopRampRate() {
        return driveMotor.getClosedLoopRampRate();
    }

    public void setDriveCurrentLimit(int limit){
        driveMotor.setSmartCurrentLimit(limit);
    }

   }