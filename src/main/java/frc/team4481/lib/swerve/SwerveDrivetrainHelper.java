package frc.team4481.lib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrivetrainHelper
{
    private SwerveModule frontLeft;
    private SwerveModule frontRight;
    private SwerveModule backLeft;
    private SwerveModule backRight;

    private SwerveDriveKinematics kinematics;

    private double maxVelocity;
    private double maxVelocityBoost;
    private boolean fieldRelative;

    public SwerveDrivetrainHelper(
            SwerveModule frontLeft,
            SwerveModule frontRight,
            SwerveModule backLeft,
            SwerveModule backRight,
            SwerveDriveKinematics kinematics
    )
    {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.kinematics = kinematics;
        //Default fieldrelative to true
        fieldRelative = true;
    }

    public void updateSwerveModuleStates(ChassisSpeeds desiredSpeeds, Rotation2d robotHeading) {
        // Convert robot  relative speeds to field relative speeds if desired
        ChassisSpeeds desiredSpeedsRobotRelative;
        if (fieldRelative) {
           desiredSpeedsRobotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeeds, robotHeading);
        } else {
            desiredSpeedsRobotRelative = desiredSpeeds;
        }

        // Convert chassis speeds to individual module states
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(desiredSpeedsRobotRelative);

        //Desaturate the individual module velocity with the right max velocity
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxVelocity);

        SwerveModule[] modules =  new SwerveModule[4];
        modules[0] = frontLeft;
        modules[1] = frontRight;
        modules[2] = backLeft;
        modules[3] = backRight;

        // Assign desired module states to modules
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);

        //Display the module states in smartdashboard
        double[] moduleTargetStates = new double[2*4];
        double[] moduleCurrStates = new double[2*4];
        for (int i = 0; i < 4; i++) {

            SwerveModuleState optimizedState = SwerveModuleState.optimize(swerveModuleStates[i], modules[i].getAbsoluteAngle());

            moduleTargetStates[i * 2] = optimizedState.angle.getRadians();
            moduleTargetStates[i * 2 + 1] = optimizedState.speedMetersPerSecond;

            moduleCurrStates[i * 2] = modules[i].getAbsoluteAngle().getRadians();
            moduleCurrStates[i * 2 + 1] = modules[i].getDriveVelocity();
        }

        SmartDashboard.putNumberArray("DT/swerve target states", moduleTargetStates);
        SmartDashboard.putNumberArray("DT/swerve current states", moduleCurrStates);
    }

    /**
     * Sets {@code ServeModule} in cross configuration
     */
    public void idleSwerveModuleStates() {
        // TODO add enum for different idle positions
        int numModules = 4;
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[numModules];
        int[] angles = {45, -45, -45, 45}; // Cross config

        for (int i = 0; i < numModules; i++) {
            Rotation2d angle = Rotation2d.fromDegrees(angles[i]);
            swerveModuleStates[i] = new SwerveModuleState(0.001, angle);
        }

        // Assign desired module states to modules
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Gets the current states of the swerve modules
     *
     * @return the current states of the swerve modules
     */
    public SwerveModuleState[] getSwerveModuleStates() {
        return new SwerveModuleState[]{
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        };
    }

    /**
     * Gets the current positions of the swerve modules
     *
     * @return the current positions of the swerve modules
     */
    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[]{
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    public void setMaxVelocity(double velocity) {
        SmartDashboard.putNumber("DT/max velocity", velocity);
        maxVelocity = velocity;
    }

    public double getMaxVelocity() {
        return maxVelocity;
    }

    public void setFieldRelative(boolean isFieldRelative) {
        fieldRelative = isFieldRelative;
    }

    public boolean getFieldRelative() {
        return fieldRelative;
    }
}