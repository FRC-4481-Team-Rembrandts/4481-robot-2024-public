package frc.team4481.robot.util;

import edu.wpi.first.math.MathUtil;

import java.util.Collections;
import java.util.List;

import static frc.team4481.robot.Constants.Pivot.PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET;

public class LookupTable {
    LookupTableConfig lookupTableConfig;

    private static LookupTable instance = new LookupTable();

    public static LookupTable getInstance() {
        return instance;
    }

    public LookupTable() {
        lookupTableConfig = new LookupTableConfig();
        List<LookupTablePreset> lookupTableConfig = this.lookupTableConfig.getLookupTableConfigs();
        ShamperSpeeds shamperspeed = new ShamperSpeeds(3500,6500,6500);

       lookupTableConfig.add(new LookupTablePreset(1.127,PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET + 48.37,shamperspeed,800,7.5,PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET +60, new ShamperSpeeds(2700, 2700,2700) )); //0 m
        lookupTableConfig.add(new LookupTablePreset(1.732,PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET + 40.0,shamperspeed,800,7.5, PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET +60,new ShamperSpeeds(2700,2700, 2700))); //0.60 m
        lookupTableConfig.add(new LookupTablePreset(2.507,PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET + 30.5,shamperspeed,800,7.5, PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET +60, new ShamperSpeeds(2700,2700, 2700))); //1.40 m
        lookupTableConfig.add(new LookupTablePreset(2.927,PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET + 27.5,shamperspeed,800,7.5, PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET +60,new ShamperSpeeds(3000, 3000,3000))); //1.81 m
        lookupTableConfig.add(new LookupTablePreset(3.20,PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET + 26.0,shamperspeed,800,7.5, PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET +50, new ShamperSpeeds(3000, 3000,3000))); //2.10 m
        lookupTableConfig.add(new LookupTablePreset(3.57,PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET + 23.5,shamperspeed,800,7.5, PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET +50,new ShamperSpeeds(3200, 3200, 3200))); //2.50 m
        lookupTableConfig.add(new LookupTablePreset(3.917,PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET + 23,shamperspeed,800,7.5, PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET +50, new ShamperSpeeds(1000, 1000, 1000))); //2.90 m
        lookupTableConfig.add(new LookupTablePreset(4.195,PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET + 21.5,shamperspeed,800,7.5, PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET +45, new ShamperSpeeds(2300, 2300,2300))); //3.20 m
        lookupTableConfig.add(new LookupTablePreset(4.40,PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET + 20.6,shamperspeed,800,7.5, PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET +45, new ShamperSpeeds(2300,2300, 2300))); //3.40 m
        lookupTableConfig.add(new LookupTablePreset(4.582,PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET + 20,shamperspeed,800,7.5, PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET +45, new ShamperSpeeds(2300,2300, 2300))); //3.60 m
        lookupTableConfig.add(new LookupTablePreset(4.785,PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET + 19.8,shamperspeed,800,7.5, PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET +45, new ShamperSpeeds(2300,2300, 2300))); //3.80 m
        lookupTableConfig.add(new LookupTablePreset(4.98,PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET + 19.3,shamperspeed,800,7.5, PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET +45, new ShamperSpeeds(2300,2300, 2300))); //4.00 m
        lookupTableConfig.add(new LookupTablePreset(5.184,PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET + 19.00,shamperspeed,800,7.5, PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET +45, new ShamperSpeeds(2300,2300, 2300))); //4.20 m

        //Add slightly lower angle at far distance to keep angle roughly constants after last measured point
        lookupTableConfig.add(new LookupTablePreset(9.30,PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET + 18.0,shamperspeed,800,7.5, PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET +45, new ShamperSpeeds(3500,4000, 4000)));
        // super cycle
        lookupTableConfig.add(new LookupTablePreset(9.50, PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET + 18.0, shamperspeed, 800, 7.5, PIVOT_ABS_ENC_ZERO_OFFSET_OFFSET +45, new ShamperSpeeds(3500,4000, 4000)));

        //Sort the table on distance
        Collections.sort(lookupTableConfig);
    }

    /**
     * Calculates an interpolated LookupTablePreset based on distance from target and the lookup table entries.
     *
     * @param distanceFromTarget the distance to the target
     * @return Interpolated LookupTablePreset.
     */
    public LookupTablePreset getLookupTablePreset(double distanceFromTarget){

        int endIndex = lookupTableConfig.getLookupTableConfigs().size()-1;

        if(distanceFromTarget <= lookupTableConfig.getLookupTableConfigs().get(0).distance()){
            return lookupTableConfig.getLookupTableConfigs().get(0);
        }

        if(distanceFromTarget >= lookupTableConfig.getLookupTableConfigs().get(endIndex).distance()){
            return lookupTableConfig.getLookupTableConfigs().get(endIndex);
        }

        return binarySearchDistance(lookupTableConfig.getLookupTableConfigs(),0, endIndex, distanceFromTarget);
    }

    /**
     * Binary search the adjacent lookup table entries for interpolation.
     *
     * @param lookupTableConfig List of all lookup table presets
     * @param startIndex Search start index
     * @param endIndex Search end index
     * @param distance distance from target
     * @return Interpolated lookup table preset
     */
    private LookupTablePreset binarySearchDistance(List<LookupTablePreset> lookupTableConfig,int startIndex, int endIndex, double distance)
    {
        int mid = startIndex + (endIndex - startIndex) / 2;
        double midIndexDistance = lookupTableConfig.get(mid).distance();

        // If the element is present at the middle
        // return itself
        if (distance == midIndexDistance)
            return lookupTableConfig.get(mid);

        // If only two elements are left
        // return the interpolated config
        if (endIndex - startIndex == 1) {
            double percentIn = (distance - this.lookupTableConfig.getLookupTableConfigs().get(startIndex).distance()) /
                    (
                            this.lookupTableConfig.getLookupTableConfigs().get(endIndex).distance() -
                                    this.lookupTableConfig.getLookupTableConfigs().get(startIndex).distance()
                    );
            return interpolateLookupTablePreset(this.lookupTableConfig.getLookupTableConfigs().get(startIndex), this.lookupTableConfig.getLookupTableConfigs().get(endIndex), percentIn);
        }

        // If element is smaller than mid, then
        // it can only be present in left subarray
        if (distance < midIndexDistance)
            return binarySearchDistance(lookupTableConfig, startIndex, mid, distance);


        // Else the element can only be present
        // in right subarray
        return binarySearchDistance(lookupTableConfig, mid, endIndex, distance);
    }

    /**
     * Interpolates all lookup table preset variables.
     *
     * @param startPreset Start value for interpolation
     * @param endPreset End value for interpolation
     * @param percentIn Percentage on linear interpolation function
     *
     * @return Interpolated lookup table preset
     */
    private LookupTablePreset interpolateLookupTablePreset(LookupTablePreset startPreset, LookupTablePreset endPreset, double percentIn) {
        ShamperSpeeds startShamperSpeeds = startPreset.shamperSpeeds();
        ShamperSpeeds endShamperSpeeds = endPreset.shamperSpeeds();

        double kickerShooterRpm = MathUtil.interpolate(startShamperSpeeds.kickerShooterRpm(), endShamperSpeeds.kickerShooterRpm(), percentIn);
        double topShooterRpm = MathUtil.interpolate(startShamperSpeeds.topShooterRpm(), endShamperSpeeds.topShooterRpm(), percentIn);
        double bottomShooterRpm = MathUtil.interpolate(startShamperSpeeds.bottomShooterRpm(), endShamperSpeeds.bottomShooterRpm(), percentIn);

        ShamperSpeeds startLopjeSpeed = startPreset.supercycleSpeeds();
        ShamperSpeeds endLopjeSpeed = endPreset.supercycleSpeeds();

        double kickerShooterRpmSupercycle = MathUtil.interpolate(startLopjeSpeed.kickerShooterRpm(), endLopjeSpeed.kickerShooterRpm(), percentIn);
        double topShooterRpmSupercycle = MathUtil.interpolate(startLopjeSpeed.topShooterRpm(), endLopjeSpeed.topShooterRpm(), percentIn);
        double bottomShooterRpmSupercycle = MathUtil.interpolate(startLopjeSpeed.bottomShooterRpm(), endLopjeSpeed.bottomShooterRpm(), percentIn);


        double pivotAngle = MathUtil.interpolate(startPreset.pivotAngle(), endPreset.pivotAngle(), percentIn);
        double distance = MathUtil.interpolate(startPreset.distance(), endPreset.distance(), percentIn);
        double ShamperMargin = MathUtil.interpolate(startPreset.ShamperMargin(), endPreset.ShamperMargin(), percentIn);
        double PivotMargin = MathUtil.interpolate(startPreset.PivotMargin(), endPreset.PivotMargin(), percentIn);
        double supercycleAngle = MathUtil.interpolate(startPreset.supercycleAngle(),endPreset.supercycleAngle(), percentIn);


        return new LookupTablePreset(distance,pivotAngle,new ShamperSpeeds(kickerShooterRpm,topShooterRpm,bottomShooterRpm), ShamperMargin, PivotMargin,supercycleAngle,new  ShamperSpeeds(kickerShooterRpmSupercycle,topShooterRpmSupercycle,bottomShooterRpmSupercycle));
    }


    /**
     * Set arbitrary lookup table preset.
     *
     * <b>MAKE SURE YOU SORT THE LIST BEFORE CALLING THIS FUNCTION</b>
     * @param pShooterConfig a sorted shooter config
     */
    public void setShooterConfig(LookupTableConfig pShooterConfig) {
        this.lookupTableConfig = pShooterConfig;
    }
}

