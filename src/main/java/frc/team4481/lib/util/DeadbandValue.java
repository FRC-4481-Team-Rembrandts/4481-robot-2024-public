package frc.team4481.lib.util;

public class DeadbandValue {
    /**
     * Based on the following graph: https://www.desmos.com/calculator/ocsopelxib
     *
     * @param x variable
     * @param deadband deadband component
     * @param linearity linear component
     * @return returns value based on deadband
     */
    public static double getDeadbandValue(double x, double deadband, double linearity){
        return (1-linearity)*(Math.pow(
                x,(Math.pow(2,deadband)+1)))
                +linearity*x;
    }
}
