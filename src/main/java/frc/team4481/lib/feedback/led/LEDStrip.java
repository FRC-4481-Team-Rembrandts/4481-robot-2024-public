package frc.team4481.lib.feedback.led;

import frc.team4481.lib.feedback.led.pattern.LEDPattern;
import frc.team4481.lib.feedback.led.pattern.RainbowPattern;

/**
 * Class to represent an LED strip with a fixed LED length and offset from the start.
 * A physical LED strip can be divided into multiple LED strip objects.
 * This allows for multiple patterns to be displayed on the same strip.
 * Each strip can have its own color, pattern, and duration.
 * @see frc.team4481.lib.feedback.led.Color
 */
public class LEDStrip {
    private final int length;
    private final int offset;

    private double patternDuration = 5;
    private Color primaryColor = new Color(12, 255, 255);
    private Color secondaryColor = new Color(102, 255, 255);
    private LEDPattern pattern = new RainbowPattern();

    /**
     * Creates a new LED strip.
     * @param length The length of the strip in LEDs.
     * @param offset The offset of the strip in LEDs.
     */
    public LEDStrip(int length, int offset) {
        this.length = length;
        this.offset = offset;
    }

    /**
     * Sets the color of the strip.
     * @param color The color to set the strip to.
     */
    @SuppressWarnings("unused")
    public void setPrimaryColor(Color color) {
        this.primaryColor = color;
    }

    /**
     * Sets the secondary of the strip.
     * @param color The color to set the strip to.
     */
    @SuppressWarnings("unused")
    public void setSecondaryColor(Color color) {
        this.secondaryColor = color;
    }


    /**
     * Sets the pattern of the strip.
     * @param pattern The pattern to set the strip to.
     */
    @SuppressWarnings("unused")
    public void setPattern(LEDPattern pattern) {
        if (this.pattern.getName() != pattern.getName()) {
            this.pattern = pattern;
        }
    }

    /**
     * Sets the time it takes for the pattern to loop.
     * @param patternDuration The pattern duration to set the strip to (in s).
     */
    @SuppressWarnings("unused")
    public void setPatternDuration(double patternDuration) {
        this.patternDuration = patternDuration;
    }

    /**
     * Gets the length of the strip.
     * @return The amount of LEDs on the strip.
     */
    public int getLength() {
        return length;
    }

    /**
     * Gets the offset of the strip.
     * @return The amount of LEDs before this strip starts.
     */
    public int getOffset() {
        return offset;
    }

    /**
     * Gets the primary color of the strip.
     * @return The color of the strip.
     */
    public Color getPrimaryColor() {
        return primaryColor;
    }

    /**
     * Gets the secondary color of the strip.
     * @return The color of the strip.
     */
    public Color getSecondaryColor() {
        return secondaryColor;
    }

    /**
     * Gets the pattern of the strip.
     * @return The pattern of the strip.
     */
    public LEDPattern getPattern() {
        return pattern;
    }

    /**
     * Gets the time it takes for the pattern to loop.
     * @return The pattern duration in s.
     */
    public double getPatternDuration() {
        return patternDuration;
    }
}
