package frc.team4481.lib.feedback.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import java.util.ArrayList;

/**
 * A controller for LED strips connected to a PWM port on the RoboRIO.
 */
@SuppressWarnings("unused")
public class PWMLEDController {
    private final AddressableLED lowLevelLEDController;
    private AddressableLEDBuffer ledBuffer;

    private final ArrayList<LEDStrip> strips = new ArrayList<>();

    /**
     * Creates a new LED controller.
     * @param port The PWM port the LED controller is connected to.
     */
    @SuppressWarnings("unused")
    public PWMLEDController(int port) {
        lowLevelLEDController = new AddressableLED(port);
    }

    /**
     * Adds an LED strip to the controller.
     * This is a relatively expensive call, so it should be avoided in the loop.
     * If a new strip is added between two existing strips,
     * all strips after the new strip will have their index shifted.
     * But the lengths and offsets will be unaffected.
     * @param strip The LED strip to be added.
     * @throws DuplicateLEDAssignmentException If the new strip overlaps with an existing strip.
     */
    public void addStrip(LEDStrip strip) throws DuplicateLEDAssignmentException {
        int index = getNewStripIndex(strip.getLength(), strip.getOffset());

        if (index == strips.size()) {
            strips.add(strip);
        } else {
            strips.add(index, strip);
        }

        updateTotalStripLength();
    }

    /**
     * Adds a new LED strip to the controller with a certain length and offset from the start.
     * This is a relatively expensive call, so it should be avoided in the loop.
     * If a new strip is added between two existing strips,
     * all strips after the new strip will have their index shifted.
     * But the lengths and offsets will be unaffected.
     * @param length The length of the strip in LEDs.
     * @param offset The offset to the start of the strip in LEDs.
     * @throws DuplicateLEDAssignmentException If the new strip overlaps with an existing strip.
     */
    public void addStrip(int length, int offset) throws DuplicateLEDAssignmentException {
        addStrip(new LEDStrip(length, offset));
    }

    /**
     * Adds a new LED strip to the controller with a certain length right after the last strip.
     * This is a relatively expensive call, so it should be avoided in the loop.
     * @param length The length of the strip in LEDs.
     * @throws DuplicateLEDAssignmentException If the new strip overlaps with an existing strip.
     */
    @SuppressWarnings("unused")
    public void addStrip(int length) throws DuplicateLEDAssignmentException {
        int offset = 0;
        if (!strips.isEmpty()) {
            offset = strips.get(strips.size() - 1).getOffset() + strips.get(strips.size() - 1).getLength();
        }
        addStrip(length, offset);
    }

    /**
     * Gets the LED strip at a certain index.
     * @param index The index of the strip.
     * @return The LED strip at the index.
     */
    @SuppressWarnings("unused")
    public LEDStrip getStrip(int index) {
        return strips.get(index);
    }

    /**
     * Removes the LED strip at a certain index.
     * This is a relatively expensive call, so it should be avoided in the loop.
     * If the strip that gets removed is not the last strip,
     * all strips after the removed strip will have their index shifted.
     * @param index The index of the strip to remove.
     */
    @SuppressWarnings("unused")
    public void removeStrip(int index) {
        strips.remove(index);
        updateTotalStripLength();
    }

    /**
     * Starts the output of the LED controller.
     * The output writes continuously.
     */
    @SuppressWarnings("unused")
    public void start() {
        lowLevelLEDController.start();
    }

    /**
     * Stops the output of the LED controller.
     */
    @SuppressWarnings("unused")
    public void stop() {
        lowLevelLEDController.stop();
    }

    /**
     * Updates the LED strips.
     * This should be called in the loop.
     */
    @SuppressWarnings("unused")
    public void updateStrips() {
        for (LEDStrip strip : strips) {
            strip.getPattern().updateBuffer(ledBuffer, strip);
        }

        lowLevelLEDController.setData(ledBuffer);
    }

    /**
     * Updates the total length of the LED strip and creates a new buffer accordingly.
     */
    private void updateTotalStripLength() {
        int totalLength = 0;
        if (!strips.isEmpty()) {
            totalLength = strips.get(strips.size() - 1).getOffset() + strips.get(strips.size() - 1).getLength();
        }

        ledBuffer = new AddressableLEDBuffer(totalLength);
        lowLevelLEDController.setLength(ledBuffer.getLength());
        lowLevelLEDController.setData(ledBuffer);
    }

    /**
     * Gets the best index of a new strip.
     * @param length The length of the new strip.
     * @param offset The offset of the new strip.
     * @throws DuplicateLEDAssignmentException If the new strip overlaps with an existing strip.
     * @return The index of the new strip.
     */
    private int getNewStripIndex(int length, int offset) throws DuplicateLEDAssignmentException {
        for (int i = 0; i < strips.size(); i++) {
            LEDStrip strip = strips.get(i);
            if (strip.getOffset() <= offset && strip.getOffset() + strip.getLength() > offset) {
                throw new DuplicateLEDAssignmentException("LED strip overlaps with existing strip at index " + i);
            }
            if (strip.getOffset() < offset && strip.getOffset() + strip.getLength() >= offset + length) {
                throw new DuplicateLEDAssignmentException("LED strip overlaps with existing strip at index " + i);
            }
        }

        for (int i = 0; i < strips.size(); i++) {
            LEDStrip strip = strips.get(i);
            if (strip.getOffset() > offset) {
                return i;
            }
        }

        return strips.size();
    }
}
