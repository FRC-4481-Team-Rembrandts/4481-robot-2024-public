package frc.team4481.lib.swerve;

public class PIDValueContainer
{
    public double kP;
    public double kI;
    public double kD;

    public PIDValueContainer(double kP, double kI, double kD)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
}
