package frc.team4481.lib.swerve;

public class SwerveModulePhysicalConstants
{
    private double driveGearRatio;
    private double wheelRadius;
    private double turnGearRatio;

    public double driveVelocityConversionFactor;
    public double drivePositionConversionFactor;
    public double turnVelocityConversionFactor;
    public double turnPositionConversionFactor;

    public SwerveModulePhysicalConstants(double driveGearRatio, double wheelRadius, double turnGearRatio)
    {
        this.driveGearRatio = driveGearRatio;
        this.wheelRadius = wheelRadius;
        this.turnGearRatio = turnGearRatio;
    }

    public void calculateConversionFactors()
    {
        // RPM to wheel velocity m/s
        driveVelocityConversionFactor =  (2 * Math.PI * wheelRadius) / (driveGearRatio * 60);

        // Rotations to meters
        drivePositionConversionFactor = 2 * Math.PI * wheelRadius / driveGearRatio;

        // RPM to radians per second
        turnVelocityConversionFactor = 2 * Math.PI / (turnGearRatio * 60);

        // rotation to radians per second
        turnPositionConversionFactor = 2 * Math.PI / turnGearRatio;
    }
}