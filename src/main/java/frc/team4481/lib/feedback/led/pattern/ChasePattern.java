package frc.team4481.lib.feedback.led.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.team4481.lib.feedback.led.Color;
import frc.team4481.lib.feedback.led.LEDStrip;

/**
 * A chase pattern that can be applied to an LED strip.
 * This pattern will override the value of the primary color.
 * The hue and saturation of the primary color are applied to the chase.
 * The secondary color is not used.
 * @see frc.team4481.lib.feedback.led.LEDStrip
 */
public class ChasePattern implements LEDPattern {
    private double valueShiftBuffer = 0;
    private double previousTime = Timer.getFPGATimestamp();
    private int firstPixelValue = 255;

    /**
     * Updates the LED buffer with the pattern.
     *
     * @param buffer The LED buffer to update.
     * @param strip  The strip to update the buffer with.
     */
    @Override
    public void updateBuffer(AddressableLEDBuffer buffer, LEDStrip strip) {
        int offset = strip.getOffset();
        int length = strip.getLength();
        double patternSeconds = strip.getPatternDuration();
        Color.HSV color = strip.getPrimaryColor().getHSV();

        // Calculate delta time
        double newTime = Timer.getFPGATimestamp();
        double deltaTime = newTime - previousTime;
        previousTime = newTime;

        // For every pixel
        for (int i = 0; i < length; i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            int value = (firstPixelValue - (i * color.value() / length)) % color.value();
            if (value < 0) value += color.value();

            // Set the value
            buffer.setHSV(i + offset, color.hue(), color.saturation(), value);
        }

        // Increase by to make the rainbow "move"
        valueShiftBuffer += ((deltaTime / patternSeconds) * color.value());
        int valueShift = (int) valueShiftBuffer;
        valueShiftBuffer -= valueShift;
        firstPixelValue -= valueShift;

        // Check bounds
        if (color.value() > 0){
            firstPixelValue %= color.value();
            if(firstPixelValue < 0) firstPixelValue += color.value();
        } else {
            firstPixelValue = 0;
        }
    }

    @Override
    public String getName() {
        return "Chase";
    }
}
