package frc.team4481.lib.feedback.led.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.team4481.lib.feedback.led.Color;
import frc.team4481.lib.feedback.led.LEDStrip;
import frc.team4481.lib.util.CountingDelay;

/**
 * A blinking pattern that can be applied to an LED strip.
 * This pattern will alternate between the primary and secondary colors.
 * @see frc.team4481.lib.feedback.led.LEDStrip
 */
public class BlinkPattern implements LEDPattern {

    private final CountingDelay blinkDelay = new CountingDelay();
    private boolean blinkPrimary = true;

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
        Color.HSV primaryColor = strip.getPrimaryColor().getHSV();
        Color.HSV secondaryColor = strip.getSecondaryColor().getHSV();
        double blinkDuration = strip.getPatternDuration();

        // Switching states should be done twice per duration
        if (blinkDelay.delay(blinkDuration / 2)) {
            blinkPrimary = !blinkPrimary;
            blinkDelay.reset();
        }

        Color.HSV color;
        if (blinkDuration <= 0) {
            color = primaryColor;
        } else {
            color = blinkPrimary ? primaryColor : secondaryColor;
        }



        // Set the LEDs to the color if blinkOn is true, otherwise set them to black
        for (int i = 0; i < length; i++) {
            buffer.setHSV(
                    offset + i,
                    color.hue(),
                    color.saturation(),
                    color.value()
            );
        }
    }

    @Override
    public String getName() {
        return "Blink";
    }
}
