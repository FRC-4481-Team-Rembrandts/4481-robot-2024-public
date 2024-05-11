package frc.team4481.lib.feedback.led.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.team4481.lib.feedback.led.LEDStrip;

/**
 * A pattern that can be applied to an LED strip.
 * @see frc.team4481.lib.feedback.led.LEDStrip
 */
public interface LEDPattern {

    /**
     * Updates the LED buffer with the pattern.
     * @param buffer The LED buffer to update.
     * @param strip The strip to update the buffer with.
     */
    void updateBuffer(AddressableLEDBuffer buffer, LEDStrip strip);

    String getName();
}
