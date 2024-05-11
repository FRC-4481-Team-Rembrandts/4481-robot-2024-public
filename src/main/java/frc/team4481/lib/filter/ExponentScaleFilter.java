package frc.team4481.lib.filter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * A filter that applies a quadratic curve to an input.
 * The input should ideally be between -1 and 1.
 * The filter applies the quadratic function to the x and y input of the joystick,
 * while making sure that the output x and y are still along a circle.
 * Squaring for example the x and y separately converts the circle into
 * a square, which is not desirable.
 */
public class ExponentScaleFilter extends FilterDecorator {

    double exponent;

    /**
     * Construct a quadratic scale filter
     * @param filter the filter to apply to the speeds
     */


    public ExponentScaleFilter(ChassisSpeedsFilter filter, double exponent) {
        super(filter);
        this.exponent = exponent;
    }



    /**
     * Returns the filtered speeds.
     * @return the filtered speeds
     */
    public ChassisSpeeds getFilteredSpeeds() {
        ChassisSpeeds speeds = tempFilter.getFilteredSpeeds();
        double x = speeds.vxMetersPerSecond;
        double y = speeds.vyMetersPerSecond;
        Translation2d inputTranslation = new Translation2d(x, y);
        double distance = inputTranslation.getNorm(); //Distance from (0,0) to the coordinate (x,y)
        Rotation2d angle = inputTranslation.getAngle(); //Angle of the vector to the coordinate (x,y)

        double newDistance = Math.pow(distance, exponent); //Square the radius to make the robot easier to control

        Translation2d newTranslation = new Translation2d(newDistance, angle);

        return new ChassisSpeeds(
                newTranslation.getX(),
                newTranslation.getY(),
                speeds.omegaRadiansPerSecond
        );
    }

    public void setExponent(double exponent){
        this.exponent = exponent;
    }


}
