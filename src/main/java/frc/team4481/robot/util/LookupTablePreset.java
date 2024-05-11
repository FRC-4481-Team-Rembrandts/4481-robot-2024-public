package frc.team4481.robot.util;


public record LookupTablePreset(double distance, double pivotAngle, ShamperSpeeds shamperSpeeds, double ShamperMargin, double PivotMargin, double supercycleAngle, ShamperSpeeds supercycleSpeeds) implements Comparable<LookupTablePreset> {

    @Override
    public int compareTo(LookupTablePreset preset) {
        return Double.compare(this.distance(), preset.distance());
    }
}

